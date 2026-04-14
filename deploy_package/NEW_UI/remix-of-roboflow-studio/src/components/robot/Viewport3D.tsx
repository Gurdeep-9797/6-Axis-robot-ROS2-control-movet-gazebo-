import React, { Suspense, useMemo, useEffect, useState, useCallback } from 'react';
import { Canvas, useThree } from '@react-three/fiber';
import { OrbitControls, Grid, PerspectiveCamera, GizmoHelper, GizmoViewport, Line } from '@react-three/drei';
import * as THREE from 'three';
import { useAppState } from '@/store/AppState';
import {
  RotateCcw, Eye, EyeOff, ShieldAlert, ShieldOff, Maximize2,
  Move, RotateCw, MousePointer, Crosshair, Scaling, Box as BoxIcon,
  Circle, Triangle, Cylinder as CylinderIcon, Layers, Sun,
  Camera, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Home
} from 'lucide-react';
import { IndustrialRobot } from './IndustrialRobot';
import { PathVisualization, WaypointMarker3D } from './PathVisualization';
import { GizmoManipulator } from './GizmoManipulator';
import { generateBlendedPath } from '@/engine/TrajectoryPlanner';

// ── Blender-style Keyboard Handler ─────────────────────────────────────
const KeyboardShortcuts = () => {
  const { setGizmoMode, gizmoMode } = useAppState();

  useEffect(() => {
    const onKeyDown = (e: KeyboardEvent) => {
      // Don't capture when typing in inputs
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;

      switch (e.key.toLowerCase()) {
        case 'g': setGizmoMode('translate'); break;   // Grab
        case 'r': setGizmoMode('rotate'); break;      // Rotate
        case 's': setGizmoMode('scale'); break;        // Scale (if type allows)
        case 'escape': setGizmoMode('off'); break;
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [setGizmoMode]);

  return null;
};

// ── Camera Presets (Numpad style) ─────────────────────────────────────
const CameraPresets = () => {
  const { camera } = useThree();
  const { cameraPreset } = useAppState();

  useEffect(() => {
    if (!cameraPreset) return;
    const dist = 3.5;
    const positions: Record<string, [number, number, number]> = {
      'front':  [0, 1, dist],
      'back':   [0, 1, -dist],
      'left':   [-dist, 1, 0],
      'right':  [dist, 1, 0],
      'top':    [0, dist, 0.01],
      'bottom': [0, -dist, 0.01],
      'perspective': [2.5, 2, 2.5],
    };
    const pos = positions[cameraPreset] || positions['perspective'];
    camera.position.set(pos[0], pos[1], pos[2]);
    camera.lookAt(0, 0.5, 0);
    camera.updateProjectionMatrix();
  }, [cameraPreset, camera]);

  return null;
};

// ── Origin Axes Indicator ─────────────────────────────────────────────
const AxisIndicator = () => (
  <group>
    <Line points={[[0,0,0],[0.5,0,0]]} color="red" lineWidth={2} />
    <Line points={[[0,0,0],[0,0.5,0]]} color="green" lineWidth={2} />
    <Line points={[[0,0,0],[0,0,0.5]]} color="blue" lineWidth={2} />
    {/* Axis labels */}
    {/* X */}
    <mesh position={[0.6, 0, 0]}>
      <sphereGeometry args={[0.015, 8, 8]} />
      <meshBasicMaterial color="red" />
    </mesh>
    {/* Y */}
    <mesh position={[0, 0.6, 0]}>
      <sphereGeometry args={[0.015, 8, 8]} />
      <meshBasicMaterial color="green" />
    </mesh>
    {/* Z */}
    <mesh position={[0, 0, 0.6]}>
      <sphereGeometry args={[0.015, 8, 8]} />
      <meshBasicMaterial color="blue" />
    </mesh>
  </group>
);

// ── Professional Industrial Table ─────────────────────────────────────
const IndustrialTable = () => (
  <group>
    {/* Table top - brushed steel surface */}
    <mesh position={[0.5, -0.005, 0.2]} receiveShadow castShadow>
      <boxGeometry args={[1.0, 0.03, 0.7]} />
      <meshStandardMaterial color="#52555a" metalness={0.75} roughness={0.25} />
    </mesh>
    {/* Table edge bevel/trim */}
    <mesh position={[0.5, -0.025, 0.2]}>
      <boxGeometry args={[1.02, 0.01, 0.72]} />
      <meshStandardMaterial color="#3a3d42" metalness={0.8} roughness={0.2} />
    </mesh>
    {/* Table support frame */}
    <mesh position={[0.5, -0.04, 0.2]}>
      <boxGeometry args={[0.96, 0.015, 0.66]} />
      <meshStandardMaterial color="#2c2e33" metalness={0.6} roughness={0.35} />
    </mesh>
    {/* 4 Heavy-duty steel legs with square profile */}
    {[[-0.02, -0.26, -0.08], [1.02, -0.26, -0.08], [-0.02, -0.26, 0.48], [1.02, -0.26, 0.48]].map((pos, i) => (
      <group key={i}>
        <mesh position={pos as [number, number, number]}>
          <boxGeometry args={[0.04, 0.48, 0.04]} />
          <meshStandardMaterial color="#3a3d42" metalness={0.7} roughness={0.3} />
        </mesh>
        {/* Floor pad */}
        <mesh position={[pos[0], -0.498, pos[2]]}>
          <cylinderGeometry args={[0.028, 0.035, 0.01, 12]} />
          <meshStandardMaterial color="#222" metalness={0.5} roughness={0.4} />
        </mesh>
      </group>
    ))}
    {/* Cross-brace for stability */}
    {[
      [[-0.02, -0.35, 0.2], [1.02, -0.35, 0.2]],   // front-back center rail
      [[0.5, -0.35, -0.08], [0.5, -0.35, 0.48]],    // left-right center rail
    ].map((pts, i) => (
      <Line key={`brace-${i}`} points={pts as [number, number, number][]} color="#3a3d42" lineWidth={3} />
    ))}
    {/* Workpiece - machined aluminum block */}
    <mesh position={[0.45, 0.035, 0.2]} castShadow>
      <boxGeometry args={[0.08, 0.04, 0.08]} />
      <meshStandardMaterial color="#7f8c8d" metalness={0.65} roughness={0.35} />
    </mesh>
    {/* Second workpiece */}
    <mesh position={[0.65, 0.025, 0.15]} castShadow>
      <cylinderGeometry args={[0.025, 0.025, 0.03, 16]} />
      <meshStandardMaterial color="#bdc3c7" metalness={0.7} roughness={0.3} />
    </mesh>
  </group>
);

// ── Workspace Envelope ────────────────────────────────────────────────
// Shows the robot's reachable workspace as a translucent dome
const WorkspaceEnvelope = ({ visible = true }: { visible?: boolean }) => {
  if (!visible) return null;
  // IRB 6700 reach is ~2.65m = 2.65 in scene units
  return (
    <group>
      <mesh position={[0, 0, 0]}>
        <sphereGeometry args={[2.65, 48, 32, 0, Math.PI * 2, 0, Math.PI / 2]} />
        <meshBasicMaterial color="#00bcd4" transparent opacity={0.03} side={THREE.DoubleSide} depthWrite={false} />
      </mesh>
      {/* Envelope wireframe ring at max height */}
      <Line
        points={Array.from({ length: 65 }, (_, i) => {
          const a = (i / 64) * Math.PI * 2;
          return [Math.cos(a) * 2.65, 0, Math.sin(a) * 2.65] as [number, number, number];
        })}
        color="#00bcd4" lineWidth={0.5}
      />
    </group>
  );
};

const SafetyFence = () => (
  <group>
    {[[-1.2, 0.3, -1.2], [1.2, 0.3, -1.2], [-1.2, 0.3, 1.2], [1.2, 0.3, 1.2]].map((pos, i) => (
      <mesh key={i} position={pos as [number, number, number]}>
        <cylinderGeometry args={[0.02, 0.02, 0.6, 8]} />
        <meshStandardMaterial color="#f39c12" metalness={0.6} roughness={0.4} />
      </mesh>
    ))}
    {/* Horizontal rails */}
    {[
      [[-1.2, 0.5, -1.2], [1.2, 0.5, -1.2]],
      [[-1.2, 0.5, 1.2], [1.2, 0.5, 1.2]],
      [[-1.2, 0.5, -1.2], [-1.2, 0.5, 1.2]],
      [[1.2, 0.5, -1.2], [1.2, 0.5, 1.2]],
    ].map((pts, i) => (
      <Line key={i} points={pts as [number, number, number][]} color="#f39c12" lineWidth={1.5} />
    ))}
  </group>
);

// Click handler for viewport picking
const GroundPlane = () => {
  const { pickingMode, setPickingMode, waypoints, setWaypoints, setSelectedNodeId, addConsoleEntry } = useAppState();

  if (pickingMode === 'none') return null;

  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}
      onClick={(e) => {
        e.stopPropagation();
        const point = e.point;
        const id = `wp-pick-${Date.now()}`;
        const newWP = {
          id, name: `WP_Picked_${waypoints.length + 1}`,
          x: Math.round(point.x * 1000), y: Math.round(point.z * 1000), z: Math.round(Math.max(0, point.y) * 1000),
          speed: 500, acceleration: 100, type: 'MoveJ' as const, zone: 'z50' as const,
          orientMode: 'quaternion' as const, qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 0, rz: 0,
        };
        setWaypoints([...waypoints, newWP]);
        setSelectedNodeId(id);
        setPickingMode('none');
        addConsoleEntry('success', `Picked waypoint at [${newWP.x}, ${newWP.y}, ${newWP.z}]`);
      }}>
      <planeGeometry args={[6, 6]} />
      <meshStandardMaterial color="#2a4060" transparent opacity={0.15} />
    </mesh>
  );
};

// ── Scene Objects (obstacles) ──────────────────────────────────────────
const SceneObjectsRenderer = () => {
  const { sceneObjects, selectedSceneObjectId, setSelectedSceneObjectId, updateSceneObject, gizmoMode } = useAppState();

  return (
    <>
      {sceneObjects?.map(obj => {
        if (!obj.visible) return null;
        const pos: [number, number, number] = [
          obj.position.x * 0.001,
          obj.position.z * 0.001,
          obj.position.y * 0.001
        ];
        const rot: [number, number, number] = [
          obj.rotation.rx,
          obj.rotation.ry,
          obj.rotation.rz
        ];
        const sc: [number, number, number] = [
          obj.scale.sx,
          obj.scale.sy,
          obj.scale.sz
        ];
        
        const w = (obj.width || 200) * 0.001;
        const h = (obj.height || 200) * 0.001;
        const d = (obj.depth || 200) * 0.001;
        const isSelected = selectedSceneObjectId === obj.id;

        return (
          <group key={obj.id}>
            <mesh 
              position={pos} 
              rotation={rot}
              scale={sc}
              onClick={(e) => {
                e.stopPropagation();
                setSelectedSceneObjectId(obj.id);
              }}
            >
              {obj.type === 'box' && <boxGeometry args={[w, h, d]} />}
              {obj.type === 'cylinder' && <cylinderGeometry args={[w / 2, w / 2, h, 16]} />}
              {obj.type === 'sphere' && <sphereGeometry args={[w / 2, 16, 16]} />}
              {obj.type === 'imported' && <torusKnotGeometry args={[w / 2, w / 6, 100, 16]} />}
              <meshStandardMaterial
                color={obj.color || '#5A5A6A'}
                transparent
                opacity={obj.opacity ?? 0.6}
                wireframe={false}
                emissive={isSelected ? '#00bcd4' : '#000000'}
                emissiveIntensity={isSelected ? 0.3 : 0}
              />
              {/* CAD Edges wireframe for precise curve selection */}
              {obj.type === 'imported' && (
                <lineSegments
                  onClick={(e) => {
                     e.stopPropagation();
                     // Map clicked curve to a new Waypoint sequence (CAD Extraction)
                     const id = `wp-cad-${Date.now()}`;
                     setWaypoints(prev => [...prev, {
                       id, name: `CAD_Curve_${prev.length + 1}`,
                       x: Math.round(e.point.x * 1000),
                       y: Math.round(e.point.z * 1000),
                       z: Math.round(Math.max(0, e.point.y) * 1000),
                       speed: 150, acceleration: 50, type: 'MoveC', zone: 'fine',
                       orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 0, rz: 0,
                       viaX: Math.round((e.point.x + 0.05) * 1000),
                       viaY: Math.round(e.point.z * 1000),
                       viaZ: Math.round((e.point.y + 0.02) * 1000)
                     }]);
                     setSelectedNodeId(id);
                     addConsoleEntry('success', `✓ CAD Curve extracted and mapped to MoveC sequence via [${id}]`);
                  }}
                >
                  <edgesGeometry args={[new THREE.TorusKnotGeometry(w / 2, w / 6, 100, 16)]} />
                  <lineBasicMaterial color={isSelected ? "#00ffff" : "#1e40af"} linewidth={2} />
                </lineSegments>
              )}
              {/* Wireframe overlay for collision objects */}
              {obj.isCollision && (
                <mesh>
                  {obj.type === 'box' && <boxGeometry args={[w * 1.01, h * 1.01, d * 1.01]} />}
                  {obj.type === 'cylinder' && <cylinderGeometry args={[w / 2 * 1.01, w / 2 * 1.01, h * 1.01, 16]} />}
                  {obj.type === 'sphere' && <sphereGeometry args={[w / 2 * 1.01, 16, 16]} />}
                  <meshBasicMaterial color={isSelected ? '#00bcd4' : '#ff4444'} wireframe transparent opacity={0.3} />
                </mesh>
              )}
            </mesh>
            
            {isSelected && gizmoMode !== 'off' && (
              <GizmoManipulator
                position={pos}
                rotation={rot}
                scale={sc}
                mode={gizmoMode}
                visible={true}
                onTransformEnd={(transform) => {
                  updateSceneObject(obj.id, {
                    position: {
                      x: Math.round(transform.position[0] * 1000),
                      y: Math.round(transform.position[2] * 1000),
                      z: Math.round(transform.position[1] * 1000) // Z is UP in UI logic
                    },
                    rotation: {
                      rx: Number(transform.rotation[0].toFixed(3)),
                      ry: Number(transform.rotation[1].toFixed(3)),
                      rz: Number(transform.rotation[2].toFixed(3))
                    },
                    scale: {
                      sx: Number(transform.scale[0].toFixed(2)),
                      sy: Number(transform.scale[1].toFixed(2)),
                      sz: Number(transform.scale[2].toFixed(2))
                    }
                  });
                }}
              />
            )}
          </group>
        );
      })}
    </>
  );
};

// ── Main Viewport Component ────────────────────────────────────────────
export const Viewport3D = () => {
  const {
    showPaths, showCollisions, setShowPaths, setShowCollisions,
    simState, simSpeed, setSimSpeed, selectedNodeId, setSelectedNodeId,
    gizmoMode, setGizmoMode, waypoints, setWaypoints,
    showBlending, showWaypoints, showTCPTrail,
    robots, selectedRobotId, setSelectedRobotId,
    jointAngles, addConsoleEntry, pickingMode,
    addSceneObject, setCameraPreset, cameraPreset,
  } = useAppState();

  const [shadingMode, setShadingMode] = useState<'solid' | 'wireframe' | 'material'>('solid');
  const [addMenuPos, setAddMenuPos] = React.useState<{x: number, y: number, point: THREE.Vector3} | null>(null);
  const [showCameraMenu, setShowCameraMenu] = useState(false);

  const pathPoints = useMemo(() => {
    if (!showPaths || waypoints.length < 2) return [];
    const targets = waypoints.map(wp => ({
      position: { x: wp.x * 0.001, y: wp.z * 0.001 + 0.5, z: wp.y * 0.001 },
      motionType: wp.type as 'MoveJ' | 'MoveL' | 'MoveC',
      speed: wp.speed, accel: wp.acceleration, zone: wp.zone,
    }));
    return generateBlendedPath(targets);
  }, [waypoints, showPaths]);

  const handleGizmoEnd = (transform: { position: [number, number, number], rotation: [number, number, number], scale: [number, number, number] }) => {
    const sel = waypoints.find(w => w.id === selectedNodeId);
    if (!sel) return;
    const newPos = transform.position;
    const newRot = transform.rotation;
    setWaypoints(waypoints.map(w =>
      w.id === sel.id ? {
        ...w,
        x: Math.round(newPos[0] * 1000),
        y: Math.round(newPos[2] * 1000),
        z: Math.round((newPos[1] - 0.5) * 1000),
        // [ANTIGRAVITY FIX] Persist rotation from gizmo back to waypoint
        rx: Number(newRot[0].toFixed(4)),
        ry: Number(newRot[1].toFixed(4)),
        rz: Number(newRot[2].toFixed(4))
      } : w
    ));
  };

  const selectedWP = waypoints.find(w => w.id === selectedNodeId);
  const gizmoPos: [number, number, number] | null = selectedWP
    ? [selectedWP.x * 0.001, selectedWP.z * 0.001 + 0.5, selectedWP.y * 0.001] : null;

  return (
    <div className="relative w-full h-full" style={{ background: 'hsl(218 30% 8%)' }}>
      {/* Main Toolbar */}
      <div className="absolute top-2 left-2 z-10 flex gap-1 p-1 rounded-lg glass-panel">
        <button className="viewport-toolbar-btn" title="Reset Camera (Numpad 0)"
          onClick={() => setCameraPreset?.('perspective')}>
          <RotateCcw size={14} />
        </button>
        <button className={`viewport-toolbar-btn ${showPaths ? 'active' : ''}`} onClick={() => setShowPaths(!showPaths)} title="Toggle Paths">
          {showPaths ? <Eye size={14} /> : <EyeOff size={14} />}
        </button>
        <button className={`viewport-toolbar-btn ${showCollisions ? 'active' : ''}`} onClick={() => setShowCollisions(!showCollisions)} title="Toggle Collisions">
          {showCollisions ? <ShieldAlert size={14} /> : <ShieldOff size={14} />}
        </button>

        <div className="w-px h-6 bg-border/30 mx-0.5" />

        {/* Gizmo mode toggles (Blender-style) */}
        <button className={`viewport-toolbar-btn ${gizmoMode === 'off' ? 'active' : ''}`}
          onClick={() => setGizmoMode('off')} title="Select (Esc)">
          <MousePointer size={14} />
        </button>
        <button className={`viewport-toolbar-btn ${gizmoMode === 'translate' ? 'active' : ''}`}
          onClick={() => setGizmoMode('translate')} title="Move (G)">
          <Move size={14} />
        </button>
        <button className={`viewport-toolbar-btn ${gizmoMode === 'rotate' ? 'active' : ''}`}
          onClick={() => setGizmoMode('rotate')} title="Rotate (R)">
          <RotateCw size={14} />
        </button>
        <button className={`viewport-toolbar-btn ${gizmoMode === 'scale' ? 'active' : ''}`}
          onClick={() => setGizmoMode('scale')} title="Scale (S)">
          <Scaling size={14} />
        </button>

        <div className="w-px h-6 bg-border/30 mx-0.5" />

        {/* Camera presets dropdown */}
        <div className="relative">
          <button className="viewport-toolbar-btn" title="Camera Views"
            onClick={() => setShowCameraMenu(!showCameraMenu)}>
            <Camera size={14} />
          </button>
          {showCameraMenu && (
            <div className="absolute top-full left-0 mt-1 z-50 glass-panel rounded-xl shadow-2xl p-1.5 min-w-[130px]"
                 style={{ border: '1px solid hsl(0 0% 100% / 0.08)' }}>
              {['perspective', 'front', 'back', 'left', 'right', 'top', 'bottom'].map(preset => (
                <button key={preset}
                  className="w-full text-left px-3 py-1.5 text-[10px] hover:bg-primary/10 rounded-lg capitalize transition-colors"
                  onClick={() => { setCameraPreset?.(preset); setShowCameraMenu(false); }}>
                  {preset}
                </button>
              ))}
            </div>
          )}
        </div>

        {/* Shading mode */}
        <button className="viewport-toolbar-btn" title="Toggle Wireframe"
          onClick={() => setShadingMode(shadingMode === 'wireframe' ? 'solid' : 'wireframe')}>
          <Layers size={14} />
        </button>
      </div>

      {/* Keyboard shortcut hints */}
      <div className="absolute bottom-2 right-2 z-10 flex flex-col gap-0.5 text-[8px] text-muted-foreground/30 font-mono pointer-events-none select-none">
        <span>MMB — Orbit</span>
        <span>Ctrl+MMB — Pan</span>
        <span>Scroll — Zoom</span>
        <span className="mt-1">G — Move</span>
        <span>R — Rotate</span>
        <span>S — Scale</span>
        <span>X/Y/Z — Axis Lock</span>
        <span>Esc — Deselect</span>
      </div>

      {/* Picking mode indicator */}
      {pickingMode !== 'none' && (
        <div className="absolute top-2 left-1/2 -translate-x-1/2 z-10 glass-panel rounded-lg px-4 py-2 flex items-center gap-2 pointer-events-none">
          <Crosshair size={14} className="text-cyan-400 animate-pulse" />
          <span className="text-xs text-cyan-400 font-medium drop-shadow-md">Click to place Waypoint Object</span>
        </div>
      )}

      {/* 3D Add Menu Popup (Blender Style) — Expanded with more shapes */}
      {addMenuPos && (
        <div className="absolute z-50 glass-panel rounded-xl flex flex-col p-2 shadow-2xl animate-fade-in border border-cyan-500/30 backdrop-blur-md"
             style={{ left: addMenuPos.x, top: addMenuPos.y }}>
             <div className="text-[10px] uppercase text-cyan-400 font-bold mb-2 ml-1 opacity-80 tracking-widest">Add Object</div>
             {[
               { icon: BoxIcon, label: 'Box', type: 'box' as const },
               { icon: CylinderIcon, label: 'Cylinder', type: 'cylinder' as const },
               { icon: Circle, label: 'Sphere', type: 'sphere' as const },
               { icon: Layers, label: 'CAD Part (Complex)', type: 'imported' as const },
             ].map(({ icon: Icon, label, type }) => (
               <button key={type}
                 className="flex items-center gap-2 px-3 py-1.5 text-xs text-foreground hover:bg-white/10 rounded my-0.5 transition-colors"
                 onClick={() => {
                   addSceneObject({
                     id: `so-${Date.now()}`, name: `${label}_${Date.now() % 100}`, type,
                     position: { x: addMenuPos.point.x*1000, y: addMenuPos.point.z*1000, z: addMenuPos.point.y*1000 },
                     rotation: { rx: 0, ry: 0, rz: 0 }, scale: { sx: 1, sy: 1, sz: 1 },
                     visible: true, locked: false, isCollision: true, color: '#5A5A6A', opacity: 0.6,
                     width: 200, depth: 200, height: 200
                   });
                   setAddMenuPos(null);
                 }}>
                 <div className="w-5 h-5 bg-gradient-to-br from-gray-500 to-gray-700 rounded shadow-sm border border-white/10 flex items-center justify-center">
                   <Icon size={12} />
                 </div>
                 <span>{label}</span>
               </button>
             ))}
             <div className="h-px bg-white/5 my-1" />
             <button className="flex items-center gap-2 px-3 py-1.5 text-xs text-foreground hover:bg-white/10 rounded my-0.5 transition-colors"
                onClick={() => {
                   const num = waypoints.length + 1;
                   setWaypoints([...waypoints, {
                     id: `wp-${Date.now()}`, name: `WP_${num}`,
                     x: addMenuPos.point.x*1000, y: addMenuPos.point.z*1000, z: addMenuPos.point.y*1000,
                     speed: 500, acceleration: 100, type: 'MoveJ', zone: 'z50',
                     orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 0, rz: 0
                   }]);
                   setAddMenuPos(null);
                }}>
                <div className="w-5 h-5 rounded-full border border-cyan-400 flex items-center justify-center text-cyan-400 shrink-0 bg-cyan-950/50">
                  <Crosshair size={12} />
                </div>
                <span>Robot Waypoint</span>
             </button>

             <div className="h-px bg-white/5 my-1" />

             <button className="flex items-center gap-2 px-3 py-1.5 text-xs text-foreground hover:bg-primary/20 rounded my-0.5 transition-colors group"
               onClick={() => {
                  setGlobalStartPoint([...jointAngles]);
                  addConsoleEntry('success', 'Global Home Point updated to current configuration');
                  setAddMenuPos(null);
               }}>
               <div className="w-5 h-5 rounded bg-primary/10 flex items-center justify-center text-primary group-hover:bg-primary/30 transition-colors">
                 <Home size={12} />
               </div>
               <div className="flex flex-col items-start leading-tight">
                 <span className="font-semibold text-primary">Set as Home</span>
                 <span className="text-[8px] opacity-50">Saves current joints</span>
               </div>
             </button>
        </div>
      )}

      {/* Multi-robot selector */}
      {robots.length > 1 && (
        <div className="absolute top-2 right-2 z-10 flex flex-col gap-0.5 p-1 rounded-lg glass-panel">
          {robots.map(r => (
            <button key={r.id}
              className={`px-2 py-1 text-[10px] rounded-md transition-all ${
                selectedRobotId === r.id ? 'bg-primary/20 text-primary' : 'text-muted-foreground hover:text-foreground hover:bg-accent/40'
              }`}
              onClick={() => { setSelectedRobotId(r.id); addConsoleEntry('info', `Selected robot: ${r.name}`); }}>
              {r.name}
            </button>
          ))}
        </div>
      )}

      {/* Speed slider */}
      <div className="absolute bottom-2 left-2 z-10 flex items-center gap-2 px-3 py-1.5 rounded-lg glass-panel">
        <span className="text-[10px] text-muted-foreground">Speed</span>
        <input type="range" min={10} max={200} value={simSpeed} onChange={e => setSimSpeed(Number(e.target.value))}
          className="w-20 h-1 accent-primary" />
        <span className="text-[10px] text-foreground w-8">{simSpeed}%</span>
      </div>

      {/* Collision indicator */}
      {showCollisions && (
        <div className="absolute top-14 right-2 z-10 px-3 py-1.5 rounded-lg glass-panel text-[10px] font-medium"
          style={{ color: 'hsl(var(--status-ok))' }}>
          ✓ No Collisions
        </div>
      )}

      <Canvas shadows onContextMenu={(e) => e.preventDefault()} onPointerMissed={() => { setGizmoMode('off'); setAddMenuPos(null); }}>
        <PerspectiveCamera makeDefault position={[2.5, 2, 2.5]} fov={45} />
        {/* SolidWorks-style navigation: MMB=rotate, Ctrl+MMB=pan, scroll=zoom */}
        <OrbitControls
          enableDamping dampingFactor={0.08}
          mouseButtons={{ LEFT: THREE.MOUSE.LEFT, MIDDLE: THREE.MOUSE.MIDDLE, RIGHT: THREE.MOUSE.RIGHT }}
          rotateSpeed={0.8}
          panSpeed={1.0}
          zoomSpeed={1.2}
          enablePan={true}
        />
        <CameraPresets />
        <KeyboardShortcuts />

        {/* Improved lighting for better visibility */}
        <ambientLight intensity={0.35} />
        <directionalLight position={[5, 8, 5]} intensity={1.0} castShadow shadow-mapSize={[2048, 2048]} />
        <directionalLight position={[-3, 4, -3]} intensity={0.4} color="#6090c0" />
        <pointLight position={[0, 3, 0]} intensity={0.2} color="#ffffff" />
        <hemisphereLight args={['#4a6ea0', '#1a1a2e', 0.4]} />

        {/* Subtle fog for depth perception */}
        <fog attach="fog" args={['#0d1117', 6, 20]} />

        <Suspense fallback={null}>
          <IndustrialRobot jointAngles={jointAngles} selected={selectedRobotId === 'robot-1'} simRunning={simState === 'running'} color={robots[0]?.color} />
          {robots.length > 1 && (
            <IndustrialRobot jointAngles={[0, -0.2, 0.3, 0, -0.4, 0]} selected={selectedRobotId === 'robot-2'}
              simRunning={simState === 'running'} color={robots[1]?.color} basePosition={[2, 0, 0]} />
          )}

          <IndustrialTable />
          <SafetyFence />
          <WorkspaceEnvelope visible={showCollisions} />
          <GroundPlane />
          <AxisIndicator />
          <SceneObjectsRenderer />

          {/* Invisible event catcher plane for right clicks */}
          <mesh rotation={[-Math.PI/2, 0, 0]} position={[0, -0.01, 0]} visible={false}
            onContextMenu={(e) => {
              e.stopPropagation();
              setAddMenuPos({ x: e.clientX, y: e.clientY, point: e.point });
            }}>
            <planeGeometry args={[100, 100]} />
          </mesh>

          {showPaths && pathPoints.length > 0 && (
            <PathVisualization pathPoints={pathPoints} showBlending={showBlending} showTCPTrail={showTCPTrail} />
          )}

          {showPaths && showWaypoints && waypoints.map(wp => (
            <WaypointMarker3D key={wp.id}
              position={[wp.x * 0.001, wp.z * 0.001 + 0.5, wp.y * 0.001]}
              name={wp.name} active={selectedNodeId === wp.id} motionType={wp.type} />
          ))}

          {gizmoPos && gizmoMode !== 'off' && (
            <GizmoManipulator position={gizmoPos} mode={gizmoMode as any} onTransformEnd={handleGizmoEnd} visible={true} />
          )}

          <Grid position={[0, 0, 0]} args={[10, 10]} cellSize={0.5} cellThickness={0.5} cellColor="#1e2d3d"
            sectionSize={2} sectionThickness={1} sectionColor="#253545" fadeDistance={10} infiniteGrid />

          {/* GizmoHelper — 3D orientation cube in corner (like Blender) */}
          <GizmoHelper alignment="bottom-right" margin={[60, 60]}>
            <GizmoViewport labelColor="white" axisHeadScale={0.8} />
          </GizmoHelper>
        </Suspense>
      </Canvas>
    </div>
  );
};
