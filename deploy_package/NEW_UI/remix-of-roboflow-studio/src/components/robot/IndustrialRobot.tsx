import React, { useRef, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

interface IndustrialRobotProps {
  jointAngles: number[];
  selected: boolean;
  simRunning: boolean;
  color?: string;
  basePosition?: [number, number, number];
}

// Rounded cylinder geometry helper
const RoundedCylinder = ({ radius, height, color, metalness = 0.7, roughness = 0.3 }: {
  radius: number; height: number; color: string; metalness?: number; roughness?: number;
}) => (
  <mesh>
    <cylinderGeometry args={[radius, radius, height, 32]} />
    <meshStandardMaterial color={color} metalness={metalness} roughness={roughness} />
  </mesh>
);

// Joint housing — rounded capsule shape
const JointHousing = ({ radius, color, emissive }: { radius: number; color: string; emissive?: string }) => (
  <mesh>
    <sphereGeometry args={[radius, 24, 24]} />
    <meshStandardMaterial
      color={color}
      metalness={0.6}
      roughness={0.35}
      emissive={emissive || '#000000'}
      emissiveIntensity={emissive ? 0.15 : 0}
    />
  </mesh>
);

// Cable bundle detail
const CableBundle = ({ path }: { path: [number, number, number][] }) => {
  const curve = useMemo(() => {
    const points = path.map(p => new THREE.Vector3(...p));
    return new THREE.CatmullRomCurve3(points);
  }, [path]);

  const geometry = useMemo(() => new THREE.TubeGeometry(curve, 16, 0.015, 6, false), [curve]);

  return (
    <mesh geometry={geometry}>
      <meshStandardMaterial color="#1a1a1a" metalness={0.3} roughness={0.8} />
    </mesh>
  );
};

export const IndustrialRobot: React.FC<IndustrialRobotProps> = ({
  jointAngles,
  selected,
  simRunning,
  color = '#e8e8e8',
  basePosition = [0, 0, 0],
}) => {
  const j1Ref = useRef<THREE.Group>(null);
  const j2Ref = useRef<THREE.Group>(null);
  const j3Ref = useRef<THREE.Group>(null);
  const j4Ref = useRef<THREE.Group>(null);
  const j5Ref = useRef<THREE.Group>(null);
  const j6Ref = useRef<THREE.Group>(null);
  const animTimeRef = useRef(0);

  const accentColor = selected ? '#3b9eff' : '#ff6b00';
  const bodyColor = color;
  const darkGrey = '#2a2a2a';
  const midGrey = '#4a4a4a';

  useFrame((state, delta) => {
    if (simRunning) {
      animTimeRef.current += delta;
      const t = animTimeRef.current;
      if (j1Ref.current) j1Ref.current.rotation.y = Math.sin(t * 0.4) * 0.6;
      if (j2Ref.current) j2Ref.current.rotation.z = Math.sin(t * 0.55 + 0.5) * 0.35 - 0.4;
      if (j3Ref.current) j3Ref.current.rotation.z = Math.sin(t * 0.7 + 1) * 0.45 + 0.2;
      if (j4Ref.current) j4Ref.current.rotation.y = Math.sin(t * 0.3 + 1.5) * 0.3;
      if (j5Ref.current) j5Ref.current.rotation.z = Math.sin(t * 0.6 + 2) * 0.4;
      if (j6Ref.current) j6Ref.current.rotation.y = t * 0.5;
    } else {
      // Apply static joint angles
      if (j1Ref.current) j1Ref.current.rotation.y = jointAngles[0] || 0;
      if (j2Ref.current) j2Ref.current.rotation.z = jointAngles[1] || 0;
      if (j3Ref.current) j3Ref.current.rotation.z = jointAngles[2] || 0;
      if (j4Ref.current) j4Ref.current.rotation.y = jointAngles[3] || 0;
      if (j5Ref.current) j5Ref.current.rotation.z = jointAngles[4] || 0;
      if (j6Ref.current) j6Ref.current.rotation.y = jointAngles[5] || 0;
    }
  });

  return (
    <group position={basePosition}>
      {/* ===== BASE PEDESTAL ===== */}
      {/* Floor mounting plate */}
      <mesh position={[0, 0.02, 0]}>
        <cylinderGeometry args={[0.52, 0.55, 0.04, 6]} />
        <meshStandardMaterial color={darkGrey} metalness={0.9} roughness={0.2} />
      </mesh>
      {/* Base column */}
      <mesh position={[0, 0.18, 0]}>
        <cylinderGeometry args={[0.38, 0.45, 0.28, 32]} />
        <meshStandardMaterial color={bodyColor} metalness={0.75} roughness={0.25} />
      </mesh>
      {/* Base accent ring */}
      <mesh position={[0, 0.08, 0]}>
        <torusGeometry args={[0.46, 0.02, 12, 32]} />
        <meshStandardMaterial color={accentColor} metalness={0.8} roughness={0.2} />
      </mesh>
      {/* Brand plate */}
      <mesh position={[0.39, 0.22, 0]} rotation={[0, 0, 0]}>
        <boxGeometry args={[0.01, 0.06, 0.12]} />
        <meshStandardMaterial color={accentColor} metalness={0.7} roughness={0.3} emissive={accentColor} emissiveIntensity={0.1} />
      </mesh>

      {/* ===== J1 — Base rotation ===== */}
      <group ref={j1Ref} position={[0, 0.32, 0]}>
        {/* J1 housing — large turret */}
        <mesh position={[0, 0.12, 0]}>
          <cylinderGeometry args={[0.32, 0.36, 0.24, 32]} />
          <meshStandardMaterial color={bodyColor} metalness={0.75} roughness={0.25} />
        </mesh>
        {/* J1 top cover */}
        <mesh position={[0, 0.26, 0]}>
          <cylinderGeometry args={[0.25, 0.32, 0.04, 32]} />
          <meshStandardMaterial color={midGrey} metalness={0.8} roughness={0.2} />
        </mesh>

        {/* ===== J2 — Shoulder ===== */}
        <group ref={j2Ref} position={[0, 0.32, 0]}>
          {/* Shoulder motor housing */}
          <mesh position={[0, 0, 0.08]} rotation={[Math.PI / 2, 0, 0]}>
            <cylinderGeometry args={[0.18, 0.18, 0.22, 24]} />
            <meshStandardMaterial color={midGrey} metalness={0.8} roughness={0.25} />
          </mesh>
          <JointHousing radius={0.14} color={accentColor} emissive={accentColor} />

          {/* Upper arm — main link */}
          <mesh position={[0, 0.48, 0]}>
            <boxGeometry args={[0.18, 0.84, 0.16]} />
            <meshStandardMaterial color={bodyColor} metalness={0.7} roughness={0.28} />
          </mesh>
          {/* Upper arm reinforcement ridges */}
          <mesh position={[0.1, 0.48, 0]}>
            <boxGeometry args={[0.02, 0.7, 0.1]} />
            <meshStandardMaterial color={midGrey} metalness={0.8} roughness={0.3} />
          </mesh>
          <mesh position={[-0.1, 0.48, 0]}>
            <boxGeometry args={[0.02, 0.7, 0.1]} />
            <meshStandardMaterial color={midGrey} metalness={0.8} roughness={0.3} />
          </mesh>

          {/* Cable routing along arm */}
          <CableBundle path={[[0.06, 0.05, 0.08], [0.06, 0.3, 0.1], [0.05, 0.6, 0.09], [0.04, 0.85, 0.08]]} />

          {/* ===== J3 — Elbow ===== */}
          <group ref={j3Ref} position={[0, 0.92, 0]}>
            {/* Elbow motor */}
            <mesh position={[0, 0, 0.1]} rotation={[Math.PI / 2, 0, 0]}>
              <cylinderGeometry args={[0.14, 0.14, 0.16, 24]} />
              <meshStandardMaterial color={midGrey} metalness={0.8} roughness={0.25} />
            </mesh>
            <JointHousing radius={0.12} color={accentColor} emissive={accentColor} />

            {/* Forearm */}
            <mesh position={[0, 0.32, 0]}>
              <boxGeometry args={[0.14, 0.55, 0.13]} />
              <meshStandardMaterial color={bodyColor} metalness={0.7} roughness={0.28} />
            </mesh>
            {/* Forearm detail plate */}
            <mesh position={[0.08, 0.35, 0]}>
              <boxGeometry args={[0.015, 0.35, 0.08]} />
              <meshStandardMaterial color={darkGrey} metalness={0.85} roughness={0.2} />
            </mesh>

            {/* ===== J4 — Wrist rotation ===== */}
            <group ref={j4Ref} position={[0, 0.62, 0]}>
              <JointHousing radius={0.09} color={accentColor} emissive={accentColor} />
              {/* Wrist housing */}
              <mesh position={[0, 0.12, 0]}>
                <cylinderGeometry args={[0.08, 0.1, 0.18, 20]} />
                <meshStandardMaterial color={bodyColor} metalness={0.75} roughness={0.25} />
              </mesh>

              {/* ===== J5 — Wrist bend ===== */}
              <group ref={j5Ref} position={[0, 0.22, 0]}>
                <JointHousing radius={0.07} color={midGrey} />

                {/* J5 link */}
                <mesh position={[0, 0.08, 0]}>
                  <cylinderGeometry args={[0.06, 0.07, 0.12, 20]} />
                  <meshStandardMaterial color={bodyColor} metalness={0.75} roughness={0.25} />
                </mesh>

                {/* ===== J6 — Tool flange ===== */}
                <group ref={j6Ref} position={[0, 0.15, 0]}>
                  <JointHousing radius={0.055} color={midGrey} />
                  {/* Tool flange plate */}
                  <mesh position={[0, 0.04, 0]}>
                    <cylinderGeometry args={[0.065, 0.065, 0.02, 24]} />
                    <meshStandardMaterial color={darkGrey} metalness={0.9} roughness={0.15} />
                  </mesh>
                  {/* Flange screw holes */}
                  {[0, 60, 120, 180, 240, 300].map(angle => (
                    <mesh key={angle} position={[
                      Math.cos(angle * Math.PI / 180) * 0.045,
                      0.05,
                      Math.sin(angle * Math.PI / 180) * 0.045
                    ]}>
                      <cylinderGeometry args={[0.006, 0.006, 0.015, 8]} />
                      <meshStandardMaterial color="#111" metalness={0.9} roughness={0.1} />
                    </mesh>
                  ))}

                  {/* ===== GRIPPER / END EFFECTOR ===== */}
                  <group position={[0, 0.07, 0]}>
                    {/* Gripper body */}
                    <mesh position={[0, 0.04, 0]}>
                      <boxGeometry args={[0.12, 0.06, 0.05]} />
                      <meshStandardMaterial color={darkGrey} metalness={0.8} roughness={0.2} />
                    </mesh>
                    {/* Pneumatic cylinder */}
                    <mesh position={[0, 0.04, -0.035]} rotation={[Math.PI / 2, 0, 0]}>
                      <cylinderGeometry args={[0.012, 0.012, 0.03, 12]} />
                      <meshStandardMaterial color="#555" metalness={0.9} roughness={0.15} />
                    </mesh>
                    {/* Left finger */}
                    <mesh position={[-0.05, 0.1, 0]}>
                      <boxGeometry args={[0.015, 0.08, 0.03]} />
                      <meshStandardMaterial color={midGrey} metalness={0.85} roughness={0.2} />
                    </mesh>
                    {/* Left finger tip */}
                    <mesh position={[-0.05, 0.15, 0]}>
                      <boxGeometry args={[0.012, 0.03, 0.025]} />
                      <meshStandardMaterial color={accentColor} metalness={0.7} roughness={0.3} />
                    </mesh>
                    {/* Right finger */}
                    <mesh position={[0.05, 0.1, 0]}>
                      <boxGeometry args={[0.015, 0.08, 0.03]} />
                      <meshStandardMaterial color={midGrey} metalness={0.85} roughness={0.2} />
                    </mesh>
                    {/* Right finger tip */}
                    <mesh position={[0.05, 0.15, 0]}>
                      <boxGeometry args={[0.012, 0.03, 0.025]} />
                      <meshStandardMaterial color={accentColor} metalness={0.7} roughness={0.3} />
                    </mesh>
                  </group>

                  {/* TCP indicator sphere */}
                  <mesh position={[0, 0.22, 0]}>
                    <sphereGeometry args={[0.015, 12, 12]} />
                    <meshStandardMaterial
                      color="#00ff88"
                      emissive="#00ff88"
                      emissiveIntensity={0.8}
                      transparent
                      opacity={0.8}
                    />
                  </mesh>
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
};
