import { useMemo } from 'react';
import * as THREE from 'three';
import { forwardKinematicsChain, selfCollisionMinDistance, UR5_SPEC } from '../lib/kinematics';

const PHOSPHOR = new THREE.Color('#6FFFB0');
const ALARM = new THREE.Color('#FF6B5E');
const STEEL = new THREE.Color('#5B6B66');

/**
 * Renders one robot arm at a given joint configuration `q` (array of 6
 * angles). `accentColor` tints the joints/links (used to distinguish
 * solvers in the grid view); `glowCollision` blends toward alarm-red as
 * the arm nears self-collision, matching the design system's energy
 * signal vocabulary.
 */
export function RobotArm({ q, accentColor = '#6FFFB0', glowCollision = true, scale = 3 }) {
  const chain = useMemo(() => forwardKinematicsChain(UR5_SPEC, q), [q]);
  const positions = useMemo(() => chain.map((c) => c.position), [chain]);

  const minDist = useMemo(
    () => (glowCollision ? selfCollisionMinDistance(UR5_SPEC, positions) : 1),
    [positions, glowCollision]
  );

  // Map collision proximity to a 0..1 alarm blend: comfortably clear
  // (>0.05m) stays at the accent color; tight or colliding (<=0) is pure
  // alarm red, with a smooth ramp in between.
  const alarmT = THREE.MathUtils.clamp(1 - (minDist + 0.02) / 0.07, 0, 1);
  const linkColor = useMemo(() => {
    const base = new THREE.Color(accentColor);
    return base.clone().lerp(ALARM, alarmT);
  }, [accentColor, alarmT]);

  return (
    <group scale={scale}>
      {/* base mounting plate -- small relative to the group scale so it
          doesn't read as a stray dark smudge on the floor */}
      <mesh position={[0, -0.015, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[0.06, 32]} />
        <meshStandardMaterial color="#161B16" metalness={0.3} roughness={0.6} />
      </mesh>
      <mesh position={[0, -0.014, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.055, 0.065, 32]} />
        <meshBasicMaterial color={accentColor} transparent opacity={0.5} />
      </mesh>

      {positions.slice(0, -1).map((p0, i) => {
        const p1 = positions[i + 1];
        const radius = UR5_SPEC.linkRadius[i] * 0.7;
        return (
          <Link key={`link-${i}`} start={p0} end={p1} radius={radius} color={linkColor} />
        );
      })}

      {positions.map((p, i) => (
        <mesh key={`joint-${i}`} position={p}>
          <sphereGeometry args={[UR5_SPEC.linkRadius[Math.min(i, 5)] * 0.85, 16, 16]} />
          <meshStandardMaterial
            color={i === positions.length - 1 ? '#EDEAE2' : linkColor}
            emissive={i === positions.length - 1 ? '#EDEAE2' : linkColor}
            emissiveIntensity={i === positions.length - 1 ? 0.6 : 0.25}
            metalness={0.3}
            roughness={0.4}
          />
        </mesh>
      ))}
    </group>
  );
}

function Link({ start, end, radius, color }) {
  const { position, quaternion, length } = useMemo(() => {
    const s = new THREE.Vector3(...start);
    const e = new THREE.Vector3(...end);
    const mid = s.clone().lerp(e, 0.5);
    const dir = e.clone().sub(s);
    const len = dir.length();
    const up = new THREE.Vector3(0, 1, 0);
    const quat = new THREE.Quaternion();
    if (len > 1e-6) {
      quat.setFromUnitVectors(up, dir.clone().normalize());
    }
    return { position: mid, quaternion: quat, length: len };
  }, [start, end]);

  if (length < 1e-6) return null;

  return (
    <mesh position={position} quaternion={quaternion}>
      <cylinderGeometry args={[radius, radius, length, 12]} />
      <meshStandardMaterial color={color} metalness={0.35} roughness={0.45} />
    </mesh>
  );
}
