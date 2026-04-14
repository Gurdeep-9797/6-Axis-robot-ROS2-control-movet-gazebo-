import React, { useRef, useEffect, useCallback } from 'react';
import { TransformControls } from '@react-three/drei';
import * as THREE from 'three';

type GizmoMode = 'translate' | 'rotate' | 'scale';

interface BlenderGizmoProps {
  position: [number, number, number];
  rotation?: [number, number, number];
  scale?: [number, number, number];
  mode: GizmoMode | 'off';
  onTransformEnd: (transform: { position: [number, number, number], rotation: [number, number, number], scale: [number, number, number] }) => void;
  visible: boolean;
}

/**
 * GizmoManipulator — Blender-style transform gizmo with:
 *  • Move/Rotate/Scale modes (G/R/S keyboard shortcuts)
 *  • Axis constraint (X/Y/Z keys after G/R/S)
 *  • Red=X, Green=Y, Blue=Z axis colors (Blender standard)
 *  • Real-time position update during drag
 */
export const GizmoManipulator: React.FC<BlenderGizmoProps> = ({
  position,
  rotation = [0, 0, 0],
  scale = [1, 1, 1],
  mode,
  onTransformEnd,
  visible,
}) => {
  const meshRef = useRef<THREE.Mesh>(null!);
  const controlsRef = useRef<any>(null);

  // Keep the anchor mesh synced with external values
  useEffect(() => {
    if (meshRef.current && !controlsRef.current?.dragging) {
      meshRef.current.position.set(position[0], position[1], position[2]);
      meshRef.current.rotation.set(rotation[0], rotation[1], rotation[2]);
      meshRef.current.scale.set(scale[0], scale[1], scale[2]);
    }
  }, [position, rotation, scale]);

  // Real-time callback during drag
  const handleChange = useCallback(() => {
    if (!meshRef.current) return;
    const p = meshRef.current.position;
    const r = meshRef.current.rotation;
    const s = meshRef.current.scale;
    onTransformEnd({
      position: [p.x, p.y, p.z],
      rotation: [r.x, r.y, r.z],
      scale: [s.x, s.y, s.z]
    });
  }, [onTransformEnd]);

  // Axis constraint keyboard handler
  useEffect(() => {
    if (!visible || !controlsRef.current) return;
    const ctrl = controlsRef.current;

    const onKeyDown = (e: KeyboardEvent) => {
      if (!ctrl) return;

      // Axis constraints (while gizmo is active)
      switch (e.key.toLowerCase()) {
        case 'x':
          ctrl.showX = true; ctrl.showY = false; ctrl.showZ = false;
          break;
        case 'y':
          ctrl.showX = false; ctrl.showY = true; ctrl.showZ = false;
          break;
        case 'z':
          ctrl.showX = false; ctrl.showY = false; ctrl.showZ = true;
          break;
        case 'escape':
          // Reset to show all axes
          ctrl.showX = true; ctrl.showY = true; ctrl.showZ = true;
          break;
      }
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [visible]);

  // Reset axis visibility when mode changes
  useEffect(() => {
    if (controlsRef.current) {
      controlsRef.current.showX = true;
      controlsRef.current.showY = true;
      controlsRef.current.showZ = true;
    }
  }, [mode]);

  if (!visible) return null;

  return (
    <>
      {/* Anchor mesh — semi-transparent cyan sphere */}
      <mesh ref={meshRef} position={position}>
        <sphereGeometry args={[0.018, 16, 16]} />
        <meshStandardMaterial
          color="#00d4ff"
          emissive="#00d4ff"
          emissiveIntensity={0.6}
          transparent
          opacity={0.9}
        />
      </mesh>
      {mode !== 'off' && (
        <TransformControls
          ref={controlsRef}
          object={meshRef.current || undefined}
          mode={mode as 'translate' | 'rotate' | 'scale'}
          size={0.6}
          onObjectChange={handleChange}
          showX
          showY
          showZ
        />
      )}
    </>
  );
};
