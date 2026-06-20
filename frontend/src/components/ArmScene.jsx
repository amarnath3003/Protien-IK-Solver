import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';

export function ArmScene({ children, compact = false }) {
  return (
    <Canvas
      camera={{ position: [2.4, 1.8, 2.4], fov: 40 }}
      gl={{ antialias: true }}
      dpr={[1, 2]}
    >
      <color attach="background" args={['#0B0E0C']} />
      <fog attach="fog" args={['#0B0E0C', 6, 14]} />

      <ambientLight intensity={0.45} />
      <directionalLight position={[3, 4, 2]} intensity={1.1} color="#EDEAE2" />
      <directionalLight position={[-2, 2, -3]} intensity={0.3} color="#6FFFB0" />

      <Grid
        args={[10, 10]}
        cellSize={0.25}
        cellThickness={0.5}
        cellColor="#1F2922"
        sectionSize={1}
        sectionThickness={1}
        sectionColor="#3A443F"
        fadeDistance={9}
        fadeStrength={1.5}
        position={[0, -0.001, 0]}
      />

      {children}

      <OrbitControls
        enableDamping
        dampingFactor={0.08}
        minDistance={compact ? 1.2 : 1.6}
        maxDistance={compact ? 5 : 8}
        maxPolarAngle={Math.PI * 0.52}
      />
    </Canvas>
  );
}
