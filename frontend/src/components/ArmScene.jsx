import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid, Environment, ContactShadows } from '@react-three/drei';

export function ArmScene({ children, compact = false }) {
  return (
    <Canvas
      camera={{ position: [2.4, 1.8, 2.4], fov: 40 }}
      gl={{ antialias: !compact }}
      dpr={compact ? 1 : [1, 1.5]}
      shadows={!compact}
    >
      <color attach="background" args={['#0B0E0C']} />
      {!compact && <fog attach="fog" args={['#0B0E0C', 6, 14]} />}

      <ambientLight intensity={compact ? 0.5 : 0.2} />
      <directionalLight position={[5, 10, 5]} intensity={1.5} color="#EDEAE2" castShadow={!compact} shadow-bias={-0.0001} />
      {!compact && <directionalLight position={[-5, 5, -5]} intensity={0.5} color="#6FFFB0" />}

      {!compact && <Environment preset="city" />}

      {/* frames={1} renders the shadow once (static bake) instead of every frame */}
      {!compact && (
        <ContactShadows position={[0, 0, 0]} opacity={0.6} scale={10} blur={2} far={4} resolution={256} color="#000000" frames={1} />
      )}

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
        enableDamping={!compact}
        dampingFactor={0.08}
        minDistance={compact ? 1.2 : 1.6}
        maxDistance={compact ? 5 : 8}
        maxPolarAngle={Math.PI * 0.52}
      />
    </Canvas>
  );
}
