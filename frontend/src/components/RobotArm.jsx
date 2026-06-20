import { useMemo } from 'react';
import * as THREE from 'three';
import { forwardKinematicsChain, selfCollisionMinDistance, UR5_SPEC, rotationOf, matrixToQuaternion } from '../lib/kinematics';

const ALARM = new THREE.Color('#FF6B5E');

export function RobotArm({ q, accentColor = '#6FFFB0', glowCollision = true, scale = 3 }) {
  const chain = useMemo(() => forwardKinematicsChain(UR5_SPEC, q), [q]);
  const positions = useMemo(() => chain.map((c) => c.position), [chain]);

  const minDist = useMemo(
    () => (glowCollision ? selfCollisionMinDistance(UR5_SPEC, positions) : 1),
    [positions, glowCollision]
  );

  const alarmT = THREE.MathUtils.clamp(1 - (minDist + 0.02) / 0.07, 0, 1);
  const ringColor = useMemo(() => {
    const base = new THREE.Color(accentColor);
    return base.clone().lerp(ALARM, alarmT);
  }, [accentColor, alarmT]);

  const bodyColor = '#8A9591';
  const darkMetal = '#2A302D';

  return (
    <group scale={scale}>
      {/* Base Pedestal */}
      <mesh position={[0, -0.04, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[0.08, 0.09, 0.08, 32]} />
        <meshPhysicalMaterial color={darkMetal} metalness={0.8} roughness={0.5} />
      </mesh>
      <mesh position={[0, -0.005, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[0.075, 0.085, 32]} />
        <meshStandardMaterial color={ringColor} emissive={ringColor} emissiveIntensity={1.5} toneMapped={false} />
      </mesh>

      {/* Links connecting the joints */}
      {positions.slice(0, -1).map((p0, i) => {
        const p1 = positions[i + 1];
        const radius = UR5_SPEC.linkRadius[i] * 0.6;
        return (
          <Link key={`link-${i}`} start={p0} end={p1} radius={radius} color={bodyColor} />
        );
      })}

      {/* Joint Motors */}
      {chain.map((frame, i) => {
        if (i === chain.length - 1) {
          // End Effector Tool
          const quat = matrixToQuaternion(rotationOf(frame));
          return (
            <group key={`ee`} position={frame.position} quaternion={quat}>
              <mesh position={[0, 0, 0.02]} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
                <cylinderGeometry args={[0.02, 0.035, 0.04, 16]} />
                <meshPhysicalMaterial color="#EDEAE2" metalness={0.4} roughness={0.2} emissive="#EDEAE2" emissiveIntensity={0.2} />
              </mesh>
            </group>
          );
        }

        const radius = UR5_SPEC.linkRadius[i] * 0.9;
        const length = radius * 2.2;
        const quat = matrixToQuaternion(rotationOf(frame));

        return (
          <group key={`joint-${i}`} position={frame.position} quaternion={quat}>
            {/* Motor Cylinder aligned to Z axis (rotation axis) */}
            <mesh rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[radius, radius, length, 32]} />
              <meshPhysicalMaterial color={bodyColor} metalness={0.7} roughness={0.3} clearcoat={0.4} />
            </mesh>
            {/* Motor Caps */}
            <mesh position={[0, 0, length / 2 + 0.002]} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[radius * 0.85, radius * 0.85, 0.005, 32]} />
              <meshPhysicalMaterial color={darkMetal} metalness={0.9} roughness={0.4} />
            </mesh>
            <mesh position={[0, 0, -length / 2 - 0.002]} rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
              <cylinderGeometry args={[radius * 0.85, radius * 0.85, 0.005, 32]} />
              <meshPhysicalMaterial color={darkMetal} metalness={0.9} roughness={0.4} />
            </mesh>
            {/* LED Ring */}
            <mesh position={[0, 0, length / 2 - 0.01]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[radius * 1.02, radius * 1.02, 0.005, 32]} />
              <meshStandardMaterial color={ringColor} emissive={ringColor} emissiveIntensity={1.5} toneMapped={false} />
            </mesh>
          </group>
        );
      })}
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
    <mesh position={position} quaternion={quaternion} castShadow receiveShadow>
      <cylinderGeometry args={[radius, radius, length, 16]} />
      <meshPhysicalMaterial color={color} metalness={0.5} roughness={0.4} clearcoat={0.3} />
    </mesh>
  );
}
