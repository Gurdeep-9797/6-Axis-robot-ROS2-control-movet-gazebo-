import React, { createContext, useContext, useState, ReactNode, useCallback, useRef, useEffect } from 'react';
import { ZoneType, RobotInstance } from '@/engine/MotionTypes';
import { IKMode, forwardKinematics } from '@/engine/IKSolver';
import { backendConnector } from '@/services/BackendConnector';

export type AppMode = 'edit' | 'simulate' | 'live';
export type ActivePanel = 'program' | 'blocks' | 'motion' | 'io' | 'diagnostics' | 'config' | 'settings' | 'project' | 'ros2' | 'scene' | 'fullscreen3d';
export type SimulationState = 'stopped' | 'running' | 'paused';
export type GizmoMode = 'translate' | 'rotate' | 'scale' | 'off';
export type ExecutionMode = 'single' | 'multi-sequential' | 'multi-parallel';
export type RightPanelTab = 'properties' | 'jog' | 'object';
export type JogMode = 'joint' | 'cartesian';
export type PickingMode = 'none' | 'waypoint';

export type SlotId = 'explorer' | 'activity' | 'bottom' | 'inspector';
export type ToolId = ActivePanel | 'console';

// ===== COMPLETE BLOCK TYPE SYSTEM =====
export type WhileConditionType = 'TRUE' | 'DI' | 'EXPR';
export interface WhileCondition {
  type: WhileConditionType;
  channel?: number;
  state?: number;
  expr?: string;
}

export type BlockNodeType =
  | 'MoveJ' | 'MoveL' | 'MoveC' | 'MoveAbsJ' | 'MoveExtJ' | 'SearchL' | 'Honing'
  | 'Wait' | 'If' | 'While' | 'For' | 'Break' | 'Return' | 'Goto' | 'Label'
  | 'SetDO' | 'GetDI' | 'PulseDO' | 'SetAO' | 'GetAI' | 'WaitDI'
  | 'GripperOpen' | 'GripperClose' | 'WaitGrip' | 'ToolChange' | 'ResetError' | 'StopProgram'
  | 'routine' | 'CallProc' | 'Module'
  | 'SetVar' | 'Increment' | 'LoadData'
  | 'SyncRobot' | 'WaitRobot';

export interface ProgramBlockNode {
  id: string;
  name: string;
  type: BlockNodeType;
  children?: ProgramBlockNode[];
  elseChildren?: ProgramBlockNode[];
  expanded?: boolean;
  // Motion
  waypointId?: string;
  targetMode?: 'waypoint' | 'viewport' | 'manual' | 'joint';
  x?: number; y?: number; z?: number;
  joints?: number[];
  speed?: number;
  accel?: number;
  zone?: ZoneType;
  orientMode?: 'quaternion' | 'euler';
  qw?: number; qx?: number; qy?: number; qz?: number;
  rx?: number; ry?: number; rz?: number;
  // Circular
  viaX?: number; viaY?: number; viaZ?: number;
  // Wait
  duration?: number;
  // IO
  signal?: string;
  value?: boolean;
  channel?: number;
  pulseWidth?: number;
  pulseCount?: number;
  analogValue?: number;
  timeout?: number;
  // Logic
  whileCondition?: WhileCondition;
  ifCondition?: string;
  conditionType?: 'DI' | 'AI' | 'Variable' | 'Expression';
  maxIterations?: number;
  breakOnError?: boolean;
  // For
  loopVar?: string;
  loopFrom?: number;
  loopTo?: number;
  loopStep?: number;
  // Labels / Goto
  labelName?: string;
  // Proc
  procName?: string;
  procArgs?: string[];
  // Gripper
  gripWidth?: number;
  gripForce?: number;
  gripSpeed?: number;
  toolRef?: string;
  // Variable
  varName?: string;
  varType?: 'num' | 'bool' | 'string' | 'pos' | 'robtarget';
  varValue?: string;
  incrementAmount?: number;
  // SearchL
  stopCondition?: string;
  maxDistance?: number;
  storePose?: boolean;
  // StopProgram
  reason?: string;
  // Sync
  targetRobotId?: string;
  // Breakpoint
  hasBreakpoint?: boolean;
  // Module
  moduleName?: string;
  // Honing specific
  boreDiameter?: number;
  strokeLength?: number;
  spindleRpm?: number;
  reciprocationSpeed?: number;
}

// ===== SCENE OBJECTS =====
export type SceneObjectType = 'box' | 'cylinder' | 'sphere' | 'conveyor' | 'imported';

export interface SceneObject {
  id: string;
  name: string;
  type: SceneObjectType;
  position: { x: number; y: number; z: number };
  rotation: { rx: number; ry: number; rz: number };
  scale: { sx: number; sy: number; sz: number };
  visible: boolean;
  locked: boolean;
  isCollision: boolean;
  color: string;
  opacity: number;
  // Dimensions
  width?: number;
  depth?: number;
  height?: number;
  radius?: number;
  // Conveyor
  beltSpeed?: number;
}

export interface Waypoint {
  id: string;
  name: string;
  x: number; y: number; z: number;
  speed: number;
  acceleration: number;
  type: 'MoveJ' | 'MoveL' | 'MoveC';
  zone: ZoneType;
  orientMode: 'quaternion' | 'euler';
  qw: number; qx: number; qy: number; qz: number;
  rx: number; ry: number; rz: number;
  wpType?: 'approach' | 'pick' | 'place' | 'general';
}

export interface ConsoleEntry {
  id: number;
  time: string;
  type: 'info' | 'warning' | 'error' | 'success' | 'debug';
  message: string;
}

export interface ProgramNode {
  id: string;
  name: string;
  type: 'folder' | 'routine' | 'instruction' | 'waypoint';
  children?: ProgramNode[];
  expanded?: boolean;
}

export interface IOPoint {
  id: string;
  name: string;
  type: 'digital-in' | 'digital-out' | 'analog-in' | 'analog-out';
  value: number | boolean;
}

export interface DiagnosticError {
  id: string;
  severity: 'error' | 'warning' | 'info';
  code: string;
  message: string;
  line?: number;
  nodeId?: string;
  timestamp: string;
}

// Timeline entry for execution visualization
export interface TimelineEntry {
  blockId: string;
  blockName: string;
  blockType: BlockNodeType;
  startTime: number;
  endTime?: number;
  status: 'running' | 'complete' | 'error';
}

interface AppState {
  mode: AppMode;
  setMode: (m: AppMode) => void;
  activePanel: ActivePanel;
  setActivePanel: (p: ActivePanel) => void;
  activeInspectorTab: string;
  setActiveInspectorTab: (p: string) => void;
  simState: SimulationState;
  setSimState: (s: SimulationState) => void;
  selectedNodeId: string | null;
  setSelectedNodeId: (id: string | null) => void;
  selectedBlockId: string | null;
  setSelectedBlockId: (id: string | null) => void;
  consoleEntries: ConsoleEntry[];
  addConsoleEntry: (type: ConsoleEntry['type'], message: string) => void;
  programTree: ProgramNode[];
  setProgramTree: (t: ProgramNode[]) => void;
  toggleTreeNode: (id: string) => void;
  programBlocks: ProgramBlockNode[];
  setProgramBlocks: (b: ProgramBlockNode[]) => void;
  toggleBlockNode: (id: string) => void;
  updateBlock: (id: string, patch: Partial<ProgramBlockNode>) => void;
  addBlockToParent: (parentId: string, block: ProgramBlockNode) => void;
  removeBlock: (blockId: string) => void;
  waypoints: Waypoint[];
  setWaypoints: (w: Waypoint[]) => void;
  ioPoints: IOPoint[];
  setIOPoints: (io: IOPoint[]) => void;
  toggleIO: (id: string) => void;
  diagnostics: DiagnosticError[];
  addDiagnostic: (d: Omit<DiagnosticError, 'id' | 'timestamp'>) => void;
  clearDiagnostics: () => void;
  showPaths: boolean;
  setShowPaths: (v: boolean) => void;
  showCollisions: boolean;
  setShowCollisions: (v: boolean) => void;
  simSpeed: number;
  setSimSpeed: (v: number) => void;
  breakpoints: Set<number>;
  toggleBreakpoint: (line: number) => void;
  projectName: string;
  setProjectName: (n: string) => void;
  robotType: string;
  setRobotType: (r: string) => void;
  rightPanelVisible: boolean;
  setRightPanelVisible: (v: boolean) => void;
  rightPanelTab: RightPanelTab;
  setRightPanelTab: (t: RightPanelTab) => void;
  consoleVisible: boolean;
  setConsoleVisible: (v: boolean) => void;
  codeEditorVisible: boolean;
  setCodeEditorVisible: (v: boolean) => void;
  settingsTab: string;
  setSettingsTab: (t: string) => void;
  editorMode: 'block' | 'script';
  setEditorMode: (m: 'block' | 'script') => void;
  ikMode: IKMode;
  setIkMode: (m: IKMode) => void;
  gizmoMode: GizmoMode;
  setGizmoMode: (m: GizmoMode) => void;
  showBlending: boolean;
  setShowBlending: (v: boolean) => void;
  showTCPTrail: boolean;
  setShowTCPTrail: (v: boolean) => void;
  showWaypoints: boolean;
  setShowWaypoints: (v: boolean) => void;
  robots: RobotInstance[];
  setRobots: (r: RobotInstance[]) => void;
  selectedRobotId: string;
  setSelectedRobotId: (id: string) => void;
  executionMode: ExecutionMode;
  setExecutionMode: (m: ExecutionMode) => void;
  jointAngles: number[];
  setJointAngles: (j: number[]) => void;
  jogMode: JogMode;
  setJogMode: (m: JogMode) => void;
  pickingMode: PickingMode;
  setPickingMode: (m: PickingMode) => void;
  executeBlock: (blockId: string) => void;
  runProgram: () => void;
  executingBlockId: string | null;
  jogStep: number;
  setJogStep: (s: number) => void;
  // Scene objects
  sceneObjects: SceneObject[];
  setSceneObjects: (o: SceneObject[]) => void;
  addSceneObject: (obj: SceneObject) => void;
  removeSceneObject: (id: string) => void;
  updateSceneObject: (id: string, patch: Partial<SceneObject>) => void;
  selectedSceneObjectId: string | null;
  setSelectedSceneObjectId: (id: string | null) => void;
  // Timeline
  timeline: TimelineEntry[];
  addTimelineEntry: (entry: TimelineEntry) => void;
  clearTimeline: () => void;
  // Camera
  cameraPreset: string | null;
  setCameraPreset: (preset: string) => void;
  // Block editor dialog
  editingBlockId: string | null;
  setEditingBlockId: (id: string | null) => void;
  // Debug
  debugPaused: boolean;
  setDebugPaused: (v: boolean) => void;
  variableWatch: Record<string, any>;
  setVariableWatch: (v: Record<string, any>) => void;
  // Global Start Point
  globalStartPoint: number[];
  setGlobalStartPoint: (j: number[]) => void;
  // Layout
  layoutSlots: Record<SlotId, ToolId>;
  setLayoutSlot: (slot: SlotId, tool: ToolId) => void;
}

const AppContext = createContext<AppState | null>(null);

export const useAppState = () => {
  const ctx = useContext(AppContext);
  if (!ctx) throw new Error('useAppState must be inside AppStateProvider');
  return ctx;
};

const defaultProgramBlocks: ProgramBlockNode[] = [
  {
    id: 'main', name: 'Main Program', type: 'Module', moduleName: 'PickPlace_Main', expanded: true, children: [
      {
        id: 'init', name: 'Initialize', type: 'routine', expanded: true, children: [
          { id: 'b-home', name: 'MoveJ Home', type: 'MoveJ', waypointId: 'wp1', targetMode: 'waypoint', x: 0, y: 0, z: 1500, speed: 500, accel: 100, zone: 'z50' },
          { id: 'b-gripper-off', name: 'SetDO Gripper OFF', type: 'SetDO', signal: 'DO_Gripper', value: false, channel: 0 },
          { id: 'b-setvar', name: 'Set counter = 0', type: 'SetVar', varName: 'counter', varType: 'num', varValue: '0' },
        ]
      },
      {
        id: 'loop-main', name: 'While TRUE', type: 'While', expanded: true,
        whileCondition: { type: 'TRUE' }, maxIterations: 0, breakOnError: true,
        children: [
          {
            id: 'pick-place', name: 'PickAndPlace', type: 'routine', expanded: true, children: [
              { id: 'b-approach', name: 'MoveJ Approach', type: 'MoveJ', waypointId: 'wp1', targetMode: 'waypoint', x: 500, y: 0, z: 1200, speed: 500, accel: 100, zone: 'z50' },
              { id: 'b-pick', name: 'MoveL Pick', type: 'MoveL', waypointId: 'wp2', targetMode: 'waypoint', x: 500, y: 0, z: 900, speed: 200, accel: 50, zone: 'fine' },
              { id: 'b-grip-on', name: 'Gripper Close', type: 'GripperClose', gripWidth: 40, gripForce: 50, gripSpeed: 100, toolRef: 'tool0' },
              { id: 'b-wait1', name: 'Wait 0.2s', type: 'Wait', duration: 0.2 },
              { id: 'b-lift', name: 'MoveL Lift', type: 'MoveL', targetMode: 'manual', x: 500, y: 0, z: 1200, speed: 500, accel: 100, zone: 'z50' },
              { id: 'b-place', name: 'MoveJ Place', type: 'MoveJ', waypointId: 'wp3', targetMode: 'waypoint', x: -400, y: 300, z: 1000, speed: 500, accel: 100, zone: 'z50' },
              { id: 'b-grip-off', name: 'Gripper Open', type: 'GripperOpen', gripWidth: 80, gripForce: 20, gripSpeed: 100, toolRef: 'tool0' },
              { id: 'b-incr', name: 'Increment counter', type: 'Increment', varName: 'counter', incrementAmount: 1 },
            ]
          },
          { id: 'b-wait-cycle', name: 'Wait 0.5s', type: 'Wait', duration: 0.5 },
        ]
      },
    ]
  }
];

const defaultTree: ProgramNode[] = [
  {
    id: 'main', name: 'Main Program', type: 'folder', expanded: true, children: [
      {
        id: 'init', name: 'Initialize', type: 'routine', expanded: false, children: [
          { id: 'home', name: 'MoveJ Home', type: 'instruction' },
          { id: 'set-speed', name: 'SetSpeed 500', type: 'instruction' },
        ]
      },
      {
        id: 'pick-place', name: 'PickAndPlace', type: 'routine', expanded: true, children: [
          { id: 'wp1', name: 'WP_Approach', type: 'waypoint' },
          { id: 'wp2', name: 'WP_Pick', type: 'waypoint' },
          { id: 'grip', name: 'SetDO gripper ON', type: 'instruction' },
          { id: 'wp3', name: 'WP_Place', type: 'waypoint' },
          { id: 'release', name: 'SetDO gripper OFF', type: 'instruction' },
        ]
      },
    ]
  }
];

const defaultWaypoints: Waypoint[] = [
  { id: 'wp-start', name: 'Start_Point', x: 800, y: 0, z: 1200, speed: 500, acceleration: 100, type: 'MoveJ', zone: 'z50', orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 90, rz: 0, wpType: 'approach' },
  { id: 'wp-w1', name: 'Weld_P1', x: 800, y: -300, z: 900, speed: 100, acceleration: 50, type: 'MoveL', zone: 'fine', orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 90, rz: 0, wpType: 'place' },
  { id: 'wp-w2', name: 'Weld_P2', x: 800, y: 300, z: 900, speed: 100, acceleration: 50, type: 'MoveL', zone: 'fine', orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 90, rz: 0, wpType: 'place' },
];

const defaultIO: IOPoint[] = [
  { id: 'do0', name: 'DO_Gripper', type: 'digital-out', value: false },
  { id: 'do1', name: 'DO_Conveyor', type: 'digital-out', value: true },
  { id: 'do2', name: 'DO_Light', type: 'digital-out', value: false },
  { id: 'do3', name: 'DO_Vacuum', type: 'digital-out', value: false },
  { id: 'di0', name: 'DI_PartSensor', type: 'digital-in', value: true },
  { id: 'di1', name: 'DI_SafetyGate', type: 'digital-in', value: true },
  { id: 'di2', name: 'DI_Emergency', type: 'digital-in', value: false },
  { id: 'di3', name: 'DI_GripConfirm', type: 'digital-in', value: false },
  { id: 'ai0', name: 'AI_ForceSensor', type: 'analog-in', value: 42.7 },
  { id: 'ai1', name: 'AI_Temperature', type: 'analog-in', value: 23.1 },
  { id: 'ao0', name: 'AO_Speed', type: 'analog-out', value: 75 },
  { id: 'ao1', name: 'AO_Pressure', type: 'analog-out', value: 5.2 },
];

const defaultRobots: RobotInstance[] = [
  {
    id: 'robot-1', name: 'IRB 6700', model: 'IRB 6700-235/2.65',
    baseTransform: { position: { x: 0, y: 0, z: 0 }, orientation: { qw: 1, qx: 0, qy: 0, qz: 0 } },
    jointState: { angles: [0, 0, 0, 0, 0, 0] }, program: [], color: '#e8e8e8',
  },
  {
    id: 'robot-2', name: 'IRB 4600', model: 'IRB 4600-60/2.05',
    baseTransform: { position: { x: 2000, y: 0, z: 0 }, orientation: { qw: 1, qx: 0, qy: 0, qz: 0 } },
    jointState: { angles: [0, 0, 0, 0, 0, 0] }, program: [], color: '#c8d8e8',
  },
];

const defaultSceneObjects: SceneObject[] = [
  {
    id: 'so-table', name: 'WorkTable', type: 'box',
    position: { x: 500, y: 200, z: 0 }, rotation: { rx: 0, ry: 0, rz: 0 }, scale: { sx: 1, sy: 1, sz: 1 },
    visible: true, locked: false, isCollision: true, color: '#3A3A4A', opacity: 1,
    width: 1500, depth: 800, height: 750,
  },
  {
    id: 'so-box1', name: 'Box_001', type: 'box',
    position: { x: 450, y: 200, z: 790 }, rotation: { rx: 0, ry: 0, rz: 0 }, scale: { sx: 1, sy: 1, sz: 1 },
    visible: true, locked: false, isCollision: true, color: '#c0392b', opacity: 1,
    width: 80, depth: 80, height: 60,
  },
  {
    id: 'so-cyl1', name: 'Cylinder_001', type: 'cylinder',
    position: { x: 600, y: 300, z: 790 }, rotation: { rx: 0, ry: 0, rz: 0 }, scale: { sx: 1, sy: 1, sz: 1 },
    visible: true, locked: false, isCollision: true, color: '#2980b9', opacity: 1,
    radius: 40, height: 100,
  },
];

export const AppStateProvider = ({ children }: { children: ReactNode }) => {
  const [mode, setMode] = useState<AppMode>('edit');
  const [activePanel, setActivePanel] = useState<ActivePanel>('program');
  const [activeInspectorTab, setActiveInspectorTab] = useState<string>('properties');
  const [simState, setSimState] = useState<SimulationState>('stopped');
  const [selectedNodeId, setSelectedNodeId] = useState<string | null>('wp2');
  const [selectedBlockId, setSelectedBlockId] = useState<string | null>(null);
  const [consoleEntries, setConsoleEntries] = useState<ConsoleEntry[]>([
    { id: 1, time: '10:24:01', type: 'info', message: 'RoboForge v6.0 initialized' },
    { id: 2, time: '10:24:01', type: 'success', message: 'Execution engine ready — IK: analytical + numerical' },
    { id: 3, time: '10:24:02', type: 'info', message: 'Robot model: IRB 6700-235/2.65 loaded' },
    { id: 4, time: '10:24:03', type: 'debug', message: 'Scene: WorkTable, Box_001, Cylinder_001 loaded' },
  ]);
  const [programTree, setProgramTree] = useState<ProgramNode[]>(defaultTree);
  const [programBlocks, setProgramBlocks] = useState<ProgramBlockNode[]>(defaultProgramBlocks);
  const [waypoints, setWaypoints] = useState<Waypoint[]>(defaultWaypoints);
  const [ioPoints, setIOPoints] = useState<IOPoint[]>(defaultIO);
  const [diagnostics, setDiagnostics] = useState<DiagnosticError[]>([]);
  const [showPaths, setShowPaths] = useState(true);
  const [showCollisions, setShowCollisions] = useState(false);
  const [simSpeed, setSimSpeed] = useState(100);
  const [breakpoints, setBreakpoints] = useState<Set<number>>(new Set([12]));
  const [projectName, setProjectName] = useState('PickPlace_Cell_01');
  const [robotType, setRobotType] = useState('IRB 6700-235/2.65');
  const [rightPanelVisible, setRightPanelVisible] = useState(true);
  const [rightPanelTab, setRightPanelTab] = useState<RightPanelTab>('properties');
  const [consoleVisible, setConsoleVisible] = useState(true);
  const [codeEditorVisible, setCodeEditorVisible] = useState(true);
  const [settingsTab, setSettingsTab] = useState('graphics');
  const [editorMode, setEditorMode] = useState<'block' | 'script'>('block');
  const [ikMode, setIkMode] = useState<IKMode>('offline-numerical');
  const [gizmoMode, setGizmoMode] = useState<GizmoMode>('translate');
  const [showBlending, setShowBlending] = useState(true);
  const [showTCPTrail, setShowTCPTrail] = useState(false);
  const [showWaypoints, setShowWaypoints] = useState(true);
  const [robots, setRobots] = useState<RobotInstance[]>(defaultRobots);
  const [selectedRobotId, setSelectedRobotId] = useState('robot-1');
  const [executionMode, setExecutionMode] = useState<ExecutionMode>('single');
  const [jointAngles, setJointAngles] = useState<number[]>([0, -0.3, 0.2, 0, -0.5, 0]);
  const [jogMode, setJogMode] = useState<JogMode>('joint');
  const [pickingMode, setPickingMode] = useState<PickingMode>('none');
  const [executingBlockId, setExecutingBlockId] = useState<string | null>(null);
  const [jogStep, setJogStep] = useState(0.05);
  const [sceneObjects, setSceneObjects] = useState<SceneObject[]>(defaultSceneObjects);
  const [selectedSceneObjectId, setSelectedSceneObjectId] = useState<string | null>(null);
  const [timeline, setTimeline] = useState<TimelineEntry[]>([]);
  const [editingBlockId, setEditingBlockId] = useState<string | null>(null);
  const [debugPaused, setDebugPaused] = useState(false);
  const [variableWatch, setVariableWatch] = useState<Record<string, any>>({ counter: 0, partCount: 0, cycleTime: 1.23 });
  const [globalStartPoint, setGlobalStartPoint] = useState<number[]>([0, 0, 0, 0, -90, 0]);
  const [cameraPreset, setCameraPreset] = useState<string | null>(null);
  const [layoutSlots, setLayoutSlots] = useState<Record<SlotId, ToolId>>({
    explorer: 'program',   // Left panel: Program Tree by default
    activity: 'io',        // (reserved)
    bottom: 'diagnostics', // Bottom panel: Diagnostics/Console by default
    inspector: 'config',   // Right inspector: Robot Config by default
  });
  const animRef = useRef<number | null>(null);

  useEffect(() => {
    backendConnector.publishSceneObjects(sceneObjects);
  }, [sceneObjects]);

  const addConsoleEntry = useCallback((type: ConsoleEntry['type'], message: string) => {
    const now = new Date();
    const time = now.toTimeString().slice(0, 8);
    setConsoleEntries(prev => [...prev.slice(-200), { id: prev.length + 1, time, type, message }]);
  }, []);

  const toggleTreeNode = (id: string) => {
    const toggle = (nodes: ProgramNode[]): ProgramNode[] =>
      nodes.map(n => n.id === id ? { ...n, expanded: !n.expanded } : { ...n, children: n.children ? toggle(n.children) : undefined });
    setProgramTree(toggle(programTree));
  };

  const toggleBlockNode = useCallback((id: string) => {
    const toggle = (nodes: ProgramBlockNode[]): ProgramBlockNode[] =>
      nodes.map(n => n.id === id ? { ...n, expanded: !n.expanded } : { ...n, children: n.children ? toggle(n.children) : undefined });
    setProgramBlocks(prev => toggle(prev));
  }, []);

  const updateBlock = useCallback((id: string, patch: Partial<ProgramBlockNode>) => {
    const update = (nodes: ProgramBlockNode[]): ProgramBlockNode[] =>
      nodes.map(n => n.id === id ? { ...n, ...patch } : { ...n, children: n.children ? update(n.children) : undefined });
    setProgramBlocks(prev => update(prev));
  }, []);

  const addBlockToParent = useCallback((parentId: string, block: ProgramBlockNode) => {
    const insert = (nodes: ProgramBlockNode[]): ProgramBlockNode[] =>
      nodes.map(n => {
        if (n.id === parentId) return { ...n, children: [...(n.children || []), block], expanded: true };
        return { ...n, children: n.children ? insert(n.children) : undefined };
      });
    setProgramBlocks(prev => insert(prev));
  }, []);

  const removeBlock = useCallback((blockId: string) => {
    const remove = (nodes: ProgramBlockNode[]): ProgramBlockNode[] =>
      nodes.filter(n => n.id !== blockId).map(n => ({ ...n, children: n.children ? remove(n.children) : undefined }));
    setProgramBlocks(prev => remove(prev));
  }, []);

  const toggleIO = (id: string) => {
    setIOPoints(prev => prev.map(io => io.id === id && (io.type === 'digital-out') ? { ...io, value: !io.value } : io));
  };

  const addDiagnostic = (d: Omit<DiagnosticError, 'id' | 'timestamp'>) => {
    setDiagnostics(prev => [...prev, { ...d, id: `diag-${Date.now()}`, timestamp: new Date().toTimeString().slice(0, 8) }]);
  };

  const clearDiagnostics = () => setDiagnostics([]);

  const toggleBreakpoint = (line: number) => {
    setBreakpoints(prev => {
      const next = new Set(prev);
      next.has(line) ? next.delete(line) : next.add(line);
      return next;
    });
  };

  const addSceneObject = useCallback((obj: SceneObject) => {
    setSceneObjects(prev => [...prev, obj]);
    addConsoleEntry('info', `Scene: Added ${obj.name}`);
  }, [addConsoleEntry]);

  const removeSceneObject = useCallback((id: string) => {
    setSceneObjects(prev => prev.filter(o => o.id !== id));
  }, []);

  const updateSceneObject = useCallback((id: string, patch: Partial<SceneObject>) => {
    setSceneObjects(prev => prev.map(o => o.id === id ? { ...o, ...patch } : o));
  }, []);

  const addTimelineEntry = useCallback((entry: TimelineEntry) => {
    setTimeline(prev => [...prev, entry]);
  }, []);

  const clearTimeline = useCallback(() => setTimeline([]), []);

  const setLayoutSlot = useCallback((slot: SlotId, tool: ToolId) => {
    setLayoutSlots(prev => ({ ...prev, [slot]: tool }));
  }, []);

  const animateToTarget = useCallback((targetJoints: number[], duration: number, blockName: string) => {
    return new Promise<void>((resolve) => {
      backendConnector.publishTrajectory(targetJoints, duration);
      if (backendConnector.mode === 'offline') {
        const startJoints = [...jointAngles];
        const startTime = performance.now();
        const durationMs = duration * 1000;
        const step = (now: number) => {
          const elapsed = now - startTime;
          const t = Math.min(1, elapsed / durationMs);
          const ease = t < 0.5 ? 2 * t * t : 1 - Math.pow(-2 * t + 2, 2) / 2;
          const current = startJoints.map((s, i) => s + (targetJoints[i] - s) * ease);
          setJointAngles(current);
          if (t < 1) {
            animRef.current = requestAnimationFrame(step);
          } else {
            setJointAngles(targetJoints);
            resolve();
          }
        };
        animRef.current = requestAnimationFrame(step);
      } else {
        setTimeout(() => { resolve(); }, duration * 1000);
      }
    });
  }, [backendConnector, jointAngles]);

  // [ANTIGRAVITY FIX] Use a ref for jointAngles so executeBlock always reads
  // the LIVE current joint angles, not the stale closure value.
  // This fixes the bug where each block execution started with [0,0,0,0,0,0] seeds.
  const jointAnglesRef = useRef(jointAngles);
  useEffect(() => {
    jointAnglesRef.current = jointAngles;
  }, [jointAngles]);

  const executeBlock = useCallback((blockId: string) => {
    const findBlock = (nodes: ProgramBlockNode[]): ProgramBlockNode | null => {
      for (const n of nodes) {
        if (n.id === blockId) return n;
        if (n.children) { const found = findBlock(n.children); if (found) return found; }
      }
      return null;
    };
    const block = findBlock(programBlocks);
    if (!block) return;
    setExecutingBlockId(blockId);
    addConsoleEntry('info', `▶ Executing: ${block.name}`);
    const startTime = performance.now();
    addTimelineEntry({ blockId, blockName: block.name, blockType: block.type, startTime, status: 'running' });
    const completeBlock = () => {
      setExecutingBlockId(null);
      setTimeline(prev => prev.map(t => t.blockId === blockId && t.status === 'running'
        ? { ...t, endTime: performance.now(), status: 'complete' as const } : t));
      addConsoleEntry('success', `✓ ${block.name} complete`);
    };
    if (['MoveJ', 'MoveL', 'MoveC', 'SearchL', 'MoveAbsJ'].includes(block.type)) {
      const tx = block.x || 0, ty = block.y || 0, tz = block.z || 0;
      const rx = block.rx || 0, ry = block.ry || 0, rz = block.rz || 0;
      // [ANTIGRAVITY FIX] Read from ref to get LIVE joint angles as IK seed
      const seedJoints = jointAnglesRef.current;
      backendConnector.computeIK(tx, ty, tz, rx, ry, rz, 1.0, ikMode, seedJoints).then((ikResult) => {
        if (ikResult) {
          const speed = block.speed || 500;
          const duration = Math.max(0.3, 1000 / speed);
          animateToTarget(ikResult.joints, duration, block.name).then(completeBlock);
        } else {
          // [ANTIGRAVITY FIX] Formal diagnostic on IK failure
          addDiagnostic({
            severity: 'error',
            code: 'IK_SOLVE_FAIL',
            message: `IK failed for block "${block.name}" in mode "${ikMode}". Position [${tx}, ${ty}, ${tz}] may be unreachable.`,
            line: undefined,
            nodeId: block.id
          });
          addConsoleEntry('error', `✗ IK solver (${ikMode}) failed for ${block.name}`);
          setExecutingBlockId(null);
          completeBlock();
        }
      });
    } else { completeBlock(); }
  }, [programBlocks, addConsoleEntry, animateToTarget, ioPoints, addTimelineEntry, ikMode]);

  const runProgram = useCallback(() => {
    const flatten = (nodes: ProgramBlockNode[]): ProgramBlockNode[] => {
      const result: ProgramBlockNode[] = [];
      for (const n of nodes) {
        if (n.children && ['routine', 'While', 'If', 'For', 'Module'].includes(n.type)) { result.push(...flatten(n.children)); }
        else { result.push(n); }
      }
      return result;
    };
    const executableBlocks = flatten(programBlocks).filter(b => ['MoveJ', 'MoveL', 'MoveC', 'Honing', 'MoveAbsJ', 'SearchL', 'Wait'].includes(b.type));
    clearTimeline();
    setSimState('running');
    let index = 0;
    // [ANTIGRAVITY FIX] Track current joints OUTSIDE closure so each block uses
    // the ACTUAL post-animation joint angles as IK seed for the next block.
    // This was the root cause of 'context loss' / config flips between blocks.
    let currentSeed = [...jointAngles];
    const runNext = () => {
      if (index >= executableBlocks.length) {
        setSimState('stopped');
        setExecutingBlockId(null);
        addConsoleEntry('success', `✓ Program complete (${executableBlocks.length} blocks)`);
        return;
      }
      const block = executableBlocks[index];
      index++;
      setExecutingBlockId(block.id);
      addConsoleEntry('info', `▶ [${index}/${executableBlocks.length}] ${block.name}`);
      const speed = block.speed || 500;
      const duration = Math.max(0.3, 1000 / speed);
      backendConnector.computeIK(
        block.x || 0, block.y || 0, block.z || 0,
        block.rx || 0, block.ry || 0, block.rz || 0, 1.0,
        ikMode,
        currentSeed  // ← always use live post-animation seed
      ).then((result) => {
        if (result) {
          currentSeed = result.joints; // ← update seed immediately on IK success
          animateToTarget(result.joints, duration, block.name).then(runNext);
        } else {
          // [ANTIGRAVITY FIX] Handle run again IK failure by inserting a Virtual Retract
          if (index === 1 && globalStartPoint && globalStartPoint.length === 6) {
             addConsoleEntry('warning', `⚠️ IK blocked for first move. Inserting safe retract to Global Start...`);
             currentSeed = [...globalStartPoint];
             animateToTarget(globalStartPoint, 1.0, 'Safe Retract').then(() => {
                 // Try computing the IK again after retracting
                 backendConnector.computeIK(
                   block.x || 0, block.y || 0, block.z || 0,
                   block.rx || 0, block.ry || 0, block.rz || 0, 1.0,
                   ikMode,
                   currentSeed
                 ).then((retryResult) => {
                    if (retryResult) {
                       currentSeed = retryResult.joints;
                       animateToTarget(retryResult.joints, duration, block.name).then(runNext);
                    } else {
                       addDiagnostic({ severity: 'error', code: 'IK_SOLVE_FAIL', message: `IK failed after retract for "${block.name}".`, nodeId: block.id });
                       addConsoleEntry('error', `✗ IK failed after retract for ${block.name} — stopping`);
                       setSimState('stopped');
                       setExecutingBlockId(null);
                    }
                 });
             });
          } else {
              addDiagnostic({
                severity: 'error',
                code: 'IK_SOLVE_FAIL',
                message: `Program stopped: IK failed for "${block.name}" at [${block.x}, ${block.y}, ${block.z}] using mode "${ikMode}".`,
                line: undefined,
                nodeId: block.id
              });
              addConsoleEntry('error', `✗ IK failed for ${block.name} — stopping`);
              setSimState('stopped');
              setExecutingBlockId(null);
          }
        }
      });
    };
    runNext();
  }, [programBlocks, jointAngles, animateToTarget, clearTimeline, ikMode, addConsoleEntry]);

  return (
    <AppContext.Provider value={{
      mode, setMode, activePanel, setActivePanel,
      activeInspectorTab, setActiveInspectorTab,
      simState, setSimState,
      selectedNodeId, setSelectedNodeId, selectedBlockId, setSelectedBlockId,
      consoleEntries, addConsoleEntry,
      programTree, setProgramTree, toggleTreeNode,
      programBlocks, setProgramBlocks, toggleBlockNode, updateBlock,
      addBlockToParent, removeBlock,
      waypoints, setWaypoints, ioPoints, setIOPoints, toggleIO,
      diagnostics, addDiagnostic, clearDiagnostics,
      showPaths, setShowPaths, showCollisions, setShowCollisions,
      simSpeed, setSimSpeed, breakpoints, toggleBreakpoint,
      projectName, setProjectName, robotType, setRobotType,
      rightPanelVisible, setRightPanelVisible,
      rightPanelTab, setRightPanelTab,
      consoleVisible, setConsoleVisible,
      codeEditorVisible, setCodeEditorVisible,
      settingsTab, setSettingsTab, editorMode, setEditorMode,
      ikMode, setIkMode, gizmoMode, setGizmoMode,
      showBlending, setShowBlending, showTCPTrail, setShowTCPTrail,
      showWaypoints, setShowWaypoints,
      robots, setRobots, selectedRobotId, setSelectedRobotId,
      executionMode, setExecutionMode,
      jointAngles, setJointAngles,
      jogMode, setJogMode,
      pickingMode, setPickingMode,
      executeBlock, runProgram, executingBlockId,
      jogStep, setJogStep,
      sceneObjects, setSceneObjects, addSceneObject, removeSceneObject, updateSceneObject,
      selectedSceneObjectId, setSelectedSceneObjectId,
      timeline, addTimelineEntry, clearTimeline,
      cameraPreset, setCameraPreset,
      editingBlockId, setEditingBlockId,
      debugPaused, setDebugPaused,
      variableWatch, setVariableWatch,
      globalStartPoint, setGlobalStartPoint,
      layoutSlots, setLayoutSlot,
    }}>
      {children}
    </AppContext.Provider>
  );
};
