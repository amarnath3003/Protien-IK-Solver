import { useMemo } from 'react';
import * as THREE from 'three';

/** Renders the IK target as a faceted marker with an orientation flag,
 * distinct from the arm's own joint spheres so it reads as "goal", not
 * "robot part". */
export function TargetMarker({ position, quaternion, scale = 3 }) {
  const quat = useMemo(
    () => new THREE.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]),
    [quaternion]
  );
  const pos = useMemo(() => new THREE.Vector3(...position).multiplyScalar(scale), [position, scale]);
  const size = 0.045 * scale;

  return (
    <group position={pos} quaternion={quat}>
      <mesh>
        <octahedronGeometry args={[size, 0]} />
        <meshBasicMaterial color="#E8B95C" wireframe />
      </mesh>
      <mesh>
        <octahedronGeometry args={[size * 0.55, 0]} />
        <meshStandardMaterial color="#E8B95C" emissive="#E8B95C" emissiveIntensity={0.6} transparent opacity={0.75} />
      </mesh>
      {/* orientation flag: a short bar along local +z, connected to the
          marker body so it reads as "this way" rather than a stray dot */}
      <mesh position={[0, 0, size * 1.1]} rotation={[Math.PI / 2, 0, 0]}>
        <cylinderGeometry args={[size * 0.06, size * 0.06, size * 1.2, 6]} />
        <meshStandardMaterial color="#E8B95C" emissive="#E8B95C" emissiveIntensity={0.5} />
      </mesh>
    </group>
  );
}
