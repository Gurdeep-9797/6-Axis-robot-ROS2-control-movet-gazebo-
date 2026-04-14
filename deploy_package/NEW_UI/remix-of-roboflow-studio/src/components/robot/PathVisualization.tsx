import React, { useMemo } from 'react';
import * as THREE from 'three';
import { MotionType } from '@/engine/MotionTypes';

interface PathPoint {
  x: number;
  y: number;
  z: number;
  motionType: MotionType;
}

interface PathVisualizationProps {
  pathPoints: PathPoint[];
  showBlending: boolean;
  showTCPTrail: boolean;
}

// Color map for motion types
const MOTION_COLORS: Record<MotionType, string> = {
  MoveJ: '#3b9eff',
  MoveL: '#2ecc71',
  MoveC: '#ff8c00',
};

export const PathVisualization: React.FC<PathVisualizationProps> = ({
  pathPoints,
  showBlending,
  showTCPTrail,
}) => {
  // Group consecutive points by motion type for color-coded segments
  const segments = useMemo(() => {
    if (pathPoints.length < 2) return [];

    const segs: { points: THREE.Vector3[]; color: string; type: MotionType }[] = [];
    let currentType = pathPoints[0].motionType;
    let currentPoints: THREE.Vector3[] = [new THREE.Vector3(pathPoints[0].x, pathPoints[0].y, pathPoints[0].z)];

    for (let i = 1; i < pathPoints.length; i++) {
      const p = pathPoints[i];
      const v = new THREE.Vector3(p.x, p.y, p.z);

      if (p.motionType !== currentType) {
        currentPoints.push(v);
        segs.push({ points: [...currentPoints], color: MOTION_COLORS[currentType], type: currentType });
        currentType = p.motionType;
        currentPoints = [v];
      } else {
        currentPoints.push(v);
      }
    }

    if (currentPoints.length >= 2) {
      segs.push({ points: currentPoints, color: MOTION_COLORS[currentType], type: currentType });
    }

    return segs;
  }, [pathPoints]);

  // TCP trail dots
  const trailDots = useMemo(() => {
    if (!showTCPTrail || pathPoints.length < 2) return [];
    const step = Math.max(1, Math.floor(pathPoints.length / 30));
    return pathPoints.filter((_, i) => i % step === 0);
  }, [pathPoints, showTCPTrail]);

  return (
    <group>
      {segments.map((seg, i) => {
        if (seg.points.length < 2) return null;

        const curve = new THREE.CatmullRomCurve3(seg.points);
        const tubeGeo = new THREE.TubeGeometry(curve, seg.points.length * 2, 0.008, 6, false);

        return (
          <mesh key={i} geometry={tubeGeo}>
            <meshStandardMaterial
              color={seg.color}
              emissive={seg.color}
              emissiveIntensity={0.4}
              transparent
              opacity={0.85}
            />
          </mesh>
        );
      })}

      {/* TCP trail dots */}
      {trailDots.map((p, i) => (
        <mesh key={`tcp-${i}`} position={[p.x, p.y, p.z]}>
          <sphereGeometry args={[0.012, 8, 8]} />
          <meshStandardMaterial
            color="#ffffff"
            emissive="#ffffff"
            emissiveIntensity={0.3}
            transparent
            opacity={0.5}
          />
        </mesh>
      ))}
    </group>
  );
};

// Waypoint marker with motion type indicator
export const WaypointMarker3D: React.FC<{
  position: [number, number, number];
  name: string;
  active: boolean;
  motionType: MotionType;
}> = ({ position, active, motionType }) => {
  const color = MOTION_COLORS[motionType];

  return (
    <group position={position}>
      {/* Main waypoint sphere */}
      <mesh>
        <sphereGeometry args={[0.04, 16, 16]} />
        <meshStandardMaterial
          color={active ? '#ffffff' : color}
          emissive={active ? '#ffffff' : color}
          emissiveIntensity={active ? 0.8 : 0.4}
        />
      </mesh>
      {/* Selection ring */}
      {active && (
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <torusGeometry args={[0.06, 0.005, 8, 24]} />
          <meshStandardMaterial color="#ffffff" emissive="#ffffff" emissiveIntensity={1} transparent opacity={0.7} />
        </mesh>
      )}
      {/* Ground projection line */}
      <mesh position={[0, -position[1] / 2, 0]}>
        <cylinderGeometry args={[0.003, 0.003, position[1], 6]} />
        <meshStandardMaterial color={color} transparent opacity={0.2} />
      </mesh>
      {/* Ground dot */}
      <mesh position={[0, -position[1], 0]} rotation={[Math.PI / 2, 0, 0]}>
        <circleGeometry args={[0.02, 16]} />
        <meshStandardMaterial color={color} transparent opacity={0.15} side={THREE.DoubleSide} />
      </mesh>
    </group>
  );
};
