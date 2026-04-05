import React, { useRef, Suspense } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Environment, PerspectiveCamera } from '@react-three/drei';
import { useAppState } from '@/store/AppState';
import { RotateCcw, Eye, EyeOff, ShieldAlert, ShieldOff, Maximize2 } from 'lucide-react';
import * as THREE from 'three';

const RobotArm = ({ selected, simRunning }: { selected: boolean; simRunning: boolean }) => {
  const groupRef = useRef<THREE.Group>(null);
  const joint1Ref = useRef<THREE.Group>(null);
  const joint2Ref = useRef<THREE.Group>(null);
  const joint3Ref = useRef<THREE.Group>(null);

  useFrame((state) => {
    if (simRunning && joint1Ref.current && joint2Ref.current && joint3Ref.current) {
      const t = state.clock.elapsedTime;
      joint1Ref.current.rotation.y = Math.sin(t * 0.5) * 0.5;
      joint2Ref.current.rotation.z = Math.sin(t * 0.7 + 1) * 0.3 - 0.3;
      joint3Ref.current.rotation.z = Math.sin(t * 0.9 + 2) * 0.4;
    }
  });

  const armColor = selected ? '#3b9eff' : '#e0e0e0';
  const jointColor = '#ff8c00';

  return (
    <group ref={groupRef} position={[0, 0, 0]}>
      {/* Base */}
      <mesh position={[0, 0.15, 0]}>
        <cylinderGeometry args={[0.4, 0.5, 0.3, 32]} />
        <meshStandardMaterial color="#555" metalness={0.8} roughness={0.3} />
      </mesh>

      {/* Joint 1 - Rotation */}
      <group ref={joint1Ref} position={[0, 0.3, 0]}>
        <mesh position={[0, 0.1, 0]}>
          <sphereGeometry args={[0.15, 16, 16]} />
          <meshStandardMaterial color={jointColor} metalness={0.6} roughness={0.4} />
        </mesh>

        {/* Link 1 */}
        <mesh position={[0, 0.55, 0]}>
          <boxGeometry args={[0.15, 0.8, 0.15]} />
          <meshStandardMaterial color={armColor} metalness={0.7} roughness={0.3} />
        </mesh>

        {/* Joint 2 */}
        <group ref={joint2Ref} position={[0, 0.95, 0]}>
          <mesh>
            <sphereGeometry args={[0.12, 16, 16]} />
            <meshStandardMaterial color={jointColor} metalness={0.6} roughness={0.4} />
          </mesh>

          {/* Link 2 */}
          <mesh position={[0, 0.4, 0]}>
            <boxGeometry args={[0.12, 0.7, 0.12]} />
            <meshStandardMaterial color={armColor} metalness={0.7} roughness={0.3} />
          </mesh>

          {/* Joint 3 */}
          <group ref={joint3Ref} position={[0, 0.75, 0]}>
            <mesh>
              <sphereGeometry args={[0.1, 16, 16]} />
              <meshStandardMaterial color={jointColor} metalness={0.6} roughness={0.4} />
            </mesh>

            {/* Link 3 (end effector) */}
            <mesh position={[0, 0.25, 0]}>
              <boxGeometry args={[0.08, 0.4, 0.08]} />
              <meshStandardMaterial color={armColor} metalness={0.7} roughness={0.3} />
            </mesh>

            {/* Gripper */}
            <mesh position={[-0.06, 0.5, 0]}>
              <boxGeometry args={[0.03, 0.12, 0.06]} />
              <meshStandardMaterial color="#888" metalness={0.8} roughness={0.2} />
            </mesh>
            <mesh position={[0.06, 0.5, 0]}>
              <boxGeometry args={[0.03, 0.12, 0.06]} />
              <meshStandardMaterial color="#888" metalness={0.8} roughness={0.2} />
            </mesh>
          </group>
        </group>
      </group>
    </group>
  );
};

const WaypointMarker = ({ position, name, active }: { position: [number, number, number]; name: string; active: boolean }) => (
  <group position={position}>
    <mesh>
      <sphereGeometry args={[0.05, 12, 12]} />
      <meshStandardMaterial color={active ? '#3b9eff' : '#ff8c00'} emissive={active ? '#3b9eff' : '#ff8c00'} emissiveIntensity={0.5} />
    </mesh>
    {/* Vertical line */}
    <mesh position={[0, -position[1] / 2, 0]}>
      <cylinderGeometry args={[0.005, 0.005, position[1], 8]} />
      <meshStandardMaterial color="#555" transparent opacity={0.4} />
    </mesh>
  </group>
);

const PathLine = () => {
  const points = [
    new THREE.Vector3(0.45, 0.5, 0.3),
    new THREE.Vector3(0.45, 0.5, 0.05),
    new THREE.Vector3(-0.3, 0.5, 0.05),
  ];
  const lineGeometry = new THREE.BufferGeometry().setFromPoints(points);

  return (
    <line>
      <bufferGeometry attach="geometry" {...lineGeometry} />
      <lineBasicMaterial attach="material" color="#3b9eff" transparent opacity={0.6} />
    </line>
  );
};

const WorkTable = () => (
  <mesh position={[0.5, -0.01, 0.2]} receiveShadow>
    <boxGeometry args={[0.8, 0.02, 0.6]} />
    <meshStandardMaterial color="#3a3a3a" metalness={0.5} roughness={0.5} />
  </mesh>
);

const WorkPiece = () => (
  <mesh position={[0.45, 0.04, 0.2]}>
    <boxGeometry args={[0.08, 0.06, 0.08]} />
    <meshStandardMaterial color="#c0392b" metalness={0.3} roughness={0.6} />
  </mesh>
);

export const Viewport3D = () => {
  const { showPaths, showCollisions, setShowPaths, setShowCollisions, simState, simSpeed, setSimSpeed, selectedNodeId } = useAppState();

  return (
    <div className="relative w-full h-full" style={{ background: 'hsl(var(--viewport-bg))' }}>
      {/* Toolbar */}
      <div className="absolute top-2 left-2 z-10 flex gap-1 p-1 rounded-sm border border-border" style={{ background: 'hsl(var(--panel-bg) / 0.9)' }}>
        <button className="viewport-toolbar-btn" title="Reset Camera">
          <RotateCcw size={14} />
        </button>
        <button className={`viewport-toolbar-btn ${showPaths ? 'active' : ''}`} onClick={() => setShowPaths(!showPaths)} title="Toggle Paths">
          {showPaths ? <Eye size={14} /> : <EyeOff size={14} />}
        </button>
        <button className={`viewport-toolbar-btn ${showCollisions ? 'active' : ''}`} onClick={() => setShowCollisions(!showCollisions)} title="Toggle Collisions">
          {showCollisions ? <ShieldAlert size={14} /> : <ShieldOff size={14} />}
        </button>
        <button className="viewport-toolbar-btn" title="Maximize">
          <Maximize2 size={14} />
        </button>
      </div>

      {/* Speed slider */}
      <div className="absolute bottom-2 left-2 z-10 flex items-center gap-2 px-2 py-1 rounded-sm border border-border" style={{ background: 'hsl(var(--panel-bg) / 0.9)' }}>
        <span className="text-[10px] text-muted-foreground">Speed</span>
        <input
          type="range"
          min={10} max={200} value={simSpeed}
          onChange={e => setSimSpeed(Number(e.target.value))}
          className="w-20 h-1 accent-primary"
        />
        <span className="text-[10px] text-foreground w-8">{simSpeed}%</span>
      </div>

      {/* Collision indicator */}
      {showCollisions && (
        <div className="absolute top-2 right-2 z-10 px-2 py-1 rounded-sm text-[10px] font-medium" style={{ background: 'hsl(var(--status-ok) / 0.2)', color: 'hsl(var(--status-ok))' }}>
          No Collisions
        </div>
      )}

      <Canvas shadows>
        <PerspectiveCamera makeDefault position={[2, 1.5, 2]} fov={50} />
        <OrbitControls enableDamping dampingFactor={0.05} />
        <ambientLight intensity={0.3} />
        <directionalLight position={[5, 5, 5]} intensity={0.8} castShadow />
        <pointLight position={[-3, 2, -3]} intensity={0.3} color="#3b9eff" />

        <Suspense fallback={null}>
          <RobotArm selected={selectedNodeId?.startsWith('wp') || false} simRunning={simState === 'running'} />
          <WorkTable />
          <WorkPiece />

          {showPaths && (
            <>
              <WaypointMarker position={[0.45, 0.5, 0.3]} name="WP_Approach" active={selectedNodeId === 'wp1'} />
              <WaypointMarker position={[0.45, 0.5, 0.05]} name="WP_Pick" active={selectedNodeId === 'wp2'} />
              <WaypointMarker position={[-0.3, 0.5, 0.05]} name="WP_Place" active={selectedNodeId === 'wp3'} />
              <PathLine />
            </>
          )}

          <Grid
            position={[0, 0, 0]}
            args={[10, 10]}
            cellSize={0.5}
            cellThickness={0.5}
            cellColor="#1a2332"
            sectionSize={2}
            sectionThickness={1}
            sectionColor="#1f3044"
            fadeDistance={10}
            infiniteGrid
          />
        </Suspense>
      </Canvas>
    </div>
  );
};
