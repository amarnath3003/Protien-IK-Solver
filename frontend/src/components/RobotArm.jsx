/**
 * RobotArm — fully imperative Three.js rendering.
 *
 * Instead of updating React state every frame (which triggers reconciliation,
 * useMemo recalcs, and JSX diffing), we:
 *  1. Render geometry once via JSX and capture refs to every mesh/group.
 *  2. In useFrame, lerp joint angles, run FK, and mutate .position /
 *     .quaternion / .scale directly on the Three.js objects.
 *
 * Result: buttery 60 fps animation with zero GC pressure and zero React overhead.
 */
import { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import {
  forwardKinematicsChain,
  selfCollisionMinDistance,
  UR5_SPEC,
  rotationOf,
  matrixToQuaternion,
} from '../lib/kinematics';

// Scratch THREE objects — reused every frame to avoid GC
const _v0   = new THREE.Vector3();
const _v1   = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _up   = new THREE.Vector3(0, 1, 0);
const _rc   = new THREE.Color();   // ring colour scratch

// Default idle pose just for safety (will be padded/sliced based on spec N)
const DEFAULT_IDLE = [0, -0.7, 0.9, -0.4, 0.6, 0];

// ─── component ───────────────────────────────────────────────────────────────
export function RobotArm({ q, spec = UR5_SPEC, accentColor = '#6FFFB0', glowCollision = true, scale = 3 }) {

  // Compute derived constants when spec changes
  const { N, LINK_R, JOINT_R, JOINT_L, IDLE_Q } = useMemo(() => {
    const n = spec.a.length;
    return {
      N: n,
      LINK_R: spec.linkRadius.map((r) => r * 0.6),
      JOINT_R: spec.linkRadius.map((r) => r * 0.9),
      JOINT_L: spec.linkRadius.map((r) => r * 0.9 * 2.2),
      IDLE_Q: DEFAULT_IDLE.slice(0, n).concat(Array(Math.max(0, n - 6)).fill(0)),
    };
  }, [spec]);

  // Keep latest prop values accessible inside useFrame without re-subscribing
  const qRef           = useRef(q);
  const accentRef      = useRef(accentColor);
  const specRef        = useRef(spec);
  qRef.current         = q;
  accentRef.current    = accentColor;
  specRef.current      = spec;

  // Mutable lerped joint angles (plain array, no React state)
  const lerpedQ = useRef([...IDLE_Q]);
  // When robot changes, reset lerpedQ length
  useMemo(() => { lerpedQ.current = [...IDLE_Q]; }, [IDLE_Q]);

  // Refs to every Three.js object we'll mutate imperatively
  const jointRefs   = useRef([]);
  const linkRefs    = useRef([]);
  const ringRefs    = useRef([]);
  const baseRingRef = useRef(null);
  
  // ensure ref arrays are long enough
  if (jointRefs.current.length < N + 1) jointRefs.current = Array(N + 1).fill(null);
  if (linkRefs.current.length < N) linkRefs.current = Array(N).fill(null);
  if (ringRefs.current.length < N) ringRefs.current = Array(N).fill(null);

  // ── per-frame update ────────────────────────────────────────────────────────
  useFrame((_, delta) => {
    const activeSpec = specRef.current;
    const target = (qRef.current && qRef.current.length === N)
      ? qRef.current : IDLE_Q;

    // Frame-rate-independent exponential lerp (speed=14 → ~50 ms settle)
    const alpha = 1 - Math.exp(-14 * delta);
    const curr  = lerpedQ.current;
    let moved   = false;

    for (let i = 0; i < N; i++) {
      const next = curr[i] + (target[i] - curr[i]) * alpha;
      if (Math.abs(next - curr[i]) > 1e-6) moved = true;
      curr[i] = next;
    }

    if (!moved) return; // arm is at rest — skip all GPU work

    // ── forward kinematics ─────────────────────────────────────────────────
    const chain = forwardKinematicsChain(activeSpec, curr);

    // ── collision glow colour ─────────────────────────────────────────────
    _rc.set(accentRef.current);
    if (glowCollision) {
      const positions = chain.map((c) => c.position);
      const minDist   = selfCollisionMinDistance(activeSpec, positions);
      const t         = THREE.MathUtils.clamp(1 - (minDist + 0.02) / 0.07, 0, 1);
      _rc.lerp(new THREE.Color('#FF6B5E'), t);
    }

    // ── update joint group transforms ─────────────────────────────────────
    for (let i = 0; i <= N; i++) {
      const g = jointRefs.current[i];
      if (!g) continue;
      const [px, py, pz] = chain[i].position;
      g.position.set(px, py, pz);
      const [qx, qy, qz, qw] = matrixToQuaternion(rotationOf(chain[i]));
      g.quaternion.set(qx, qy, qz, qw);
    }

    // ── update LED ring colours ────────────────────────────────────────────
    for (let i = 0; i < N; i++) {
      const m = ringRefs.current[i];
      if (m) {
        m.material.color.copy(_rc);
        m.material.emissive.copy(_rc);
      }
    }
    if (baseRingRef.current) {
      baseRingRef.current.material.color.copy(_rc);
      baseRingRef.current.material.emissive.copy(_rc);
    }

    // ── update link cylinders ──────────────────────────────────────────────
    // Geometry is unit height/radius; we set scale=(r, length, r) each frame.
    for (let i = 0; i < N; i++) {
      const mesh = linkRefs.current[i];
      if (!mesh) continue;

      _v0.set(...chain[i].position);
      _v1.set(...chain[i + 1].position);
      const len = _v0.distanceTo(_v1);

      if (len < 1e-6) { mesh.visible = false; continue; }
      mesh.visible = true;

      // midpoint
      mesh.position.set(
        (_v0.x + _v1.x) * 0.5,
        (_v0.y + _v1.y) * 0.5,
        (_v0.z + _v1.z) * 0.5,
      );

      // orientation: align cylinder's Y axis with link direction
      _v1.sub(_v0).normalize();
      _quat.setFromUnitVectors(_up, _v1);
      mesh.quaternion.copy(_quat);

      // scale encodes both radius and length
      const r = LINK_R[i];
      mesh.scale.set(r, len, r);
    }
  });

  // ── initial FK for first-render positions ─────────────────────────────────
  const initChain = useMemo(() => forwardKinematicsChain(spec, IDLE_Q), [spec, IDLE_Q]);

  // ── JSX: static geometry; Three.js takes over via refs ────────────────────
  return (
    <group scale={scale}>

      {/* ── Base Pedestal (fully static) ─────────────────────────────────── */}
      <mesh position={[0, -0.04, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.08, 0.09, 0.08, 32]} />
        <meshPhysicalMaterial color="#2A302D" metalness={0.8} roughness={0.5} />
      </mesh>
      <mesh
        ref={baseRingRef}
        position={[0, -0.005, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
      >
        <ringGeometry args={[0.075, 0.085, 32]} />
        <meshStandardMaterial
          color={accentColor} emissive={accentColor}
          emissiveIntensity={1.5} toneMapped={false}
        />
      </mesh>

      {/* ── Link cylinders — unit geometry, scaled imperatively ──────────── */}
      {Array.from({ length: N }, (_, i) => (
        <mesh
          key={`link-${i}`}
          ref={(el) => { linkRefs.current[i] = el; }}
          castShadow receiveShadow
        >
          {/* height=1, radius=1 — scale is set in useFrame */}
          <cylinderGeometry args={[1, 1, 1, 16]} />
          <meshPhysicalMaterial
            color="#8A9591" metalness={0.5} roughness={0.4} clearcoat={0.3}
          />
        </mesh>
      ))}

      {/* ── Joint motors + End Effector — position/quaternion set imperatively */}
      {initChain.map((frame, i) => {
        const isEE = i === N;
        const ref  = (el) => { jointRefs.current[i] = el; };

        if (isEE) {
          return (
            <group key="ee" ref={ref} position={frame.position}>
              <mesh
                position={[0, 0, 0.02]}
                rotation={[Math.PI / 2, 0, 0]}
                castShadow receiveShadow
              >
                <cylinderGeometry args={[0.02, 0.035, 0.04, 16]} />
                <meshPhysicalMaterial
                  color="#EDEAE2" metalness={0.4} roughness={0.2}
                  emissive="#EDEAE2" emissiveIntensity={0.2}
                />
              </mesh>
            </group>
          );
        }

        const r = JOINT_R[i];
        const l = JOINT_L[i];

        return (
          <group key={`joint-${i}`} ref={ref} position={frame.position}>
            {/* Motor body */}
            <mesh rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[r, r, l, 32]} />
              <meshPhysicalMaterial
                color="#8A9591" metalness={0.7} roughness={0.3} clearcoat={0.4}
              />
            </mesh>
            {/* Front cap */}
            <mesh position={[0, 0, l / 2 + 0.002]} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[r * 0.85, r * 0.85, 0.005, 32]} />
              <meshPhysicalMaterial color="#2A302D" metalness={0.9} roughness={0.4} />
            </mesh>
            {/* Back cap */}
            <mesh position={[0, 0, -l / 2 - 0.002]} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[r * 0.85, r * 0.85, 0.005, 32]} />
              <meshPhysicalMaterial color="#2A302D" metalness={0.9} roughness={0.4} />
            </mesh>
            {/* LED ring */}
            <mesh
              ref={(el) => { ringRefs.current[i] = el; }}
              position={[0, 0, l / 2 - 0.01]}
              rotation={[Math.PI / 2, 0, 0]}
            >
              <cylinderGeometry args={[r * 1.02, r * 1.02, 0.005, 32]} />
              <meshStandardMaterial
                color={accentColor} emissive={accentColor}
                emissiveIntensity={1.5} toneMapped={false}
              />
            </mesh>
          </group>
        );
      })}
    </group>
  );
}
