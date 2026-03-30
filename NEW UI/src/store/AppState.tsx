import React, { createContext, useContext, useState, ReactNode } from 'react';

export type AppMode = 'edit' | 'simulate' | 'live';
export type ActivePanel = 'program' | 'blocks' | 'motion' | 'io' | 'diagnostics' | 'config' | 'settings' | 'project';
export type SimulationState = 'stopped' | 'running' | 'paused';

export interface Waypoint {
  id: string;
  name: string;
  x: number; y: number; z: number;
  speed: number;
  acceleration: number;
  type: 'MoveJ' | 'MoveL';
}

export interface ConsoleEntry {
  id: number;
  time: string;
  type: 'info' | 'warning' | 'error' | 'success';
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

interface AppState {
  mode: AppMode;
  setMode: (m: AppMode) => void;
  activePanel: ActivePanel;
  setActivePanel: (p: ActivePanel) => void;
  simState: SimulationState;
  setSimState: (s: SimulationState) => void;
  selectedNodeId: string | null;
  setSelectedNodeId: (id: string | null) => void;
  consoleEntries: ConsoleEntry[];
  addConsoleEntry: (type: ConsoleEntry['type'], message: string) => void;
  programTree: ProgramNode[];
  setProgramTree: (t: ProgramNode[]) => void;
  toggleTreeNode: (id: string) => void;
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
  consoleVisible: boolean;
  setConsoleVisible: (v: boolean) => void;
  codeEditorVisible: boolean;
  setCodeEditorVisible: (v: boolean) => void;
  settingsTab: string;
  setSettingsTab: (t: string) => void;
  editorMode: 'block' | 'script';
  setEditorMode: (m: 'block' | 'script') => void;
}

const AppContext = createContext<AppState | null>(null);

export const useAppState = () => {
  const ctx = useContext(AppContext);
  if (!ctx) throw new Error('useAppState must be inside AppStateProvider');
  return ctx;
};

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
      {
        id: 'loop1', name: 'CycleLoop', type: 'routine', expanded: false, children: [
          { id: 'while1', name: 'While running', type: 'instruction' },
          { id: 'call-pp', name: 'Call PickAndPlace', type: 'instruction' },
          { id: 'wait1', name: 'Wait 0.5s', type: 'instruction' },
        ]
      },
    ]
  }
];

const defaultWaypoints: Waypoint[] = [
  { id: 'wp1', name: 'WP_Approach', x: 450, y: 200, z: 300, speed: 500, acceleration: 100, type: 'MoveJ' },
  { id: 'wp2', name: 'WP_Pick', x: 450, y: 200, z: 50, speed: 200, acceleration: 50, type: 'MoveL' },
  { id: 'wp3', name: 'WP_Place', x: -300, y: 400, z: 50, speed: 200, acceleration: 50, type: 'MoveL' },
];

const defaultIO: IOPoint[] = [
  { id: 'do1', name: 'DO_Gripper', type: 'digital-out', value: false },
  { id: 'do2', name: 'DO_Conveyor', type: 'digital-out', value: true },
  { id: 'do3', name: 'DO_Light', type: 'digital-out', value: false },
  { id: 'di1', name: 'DI_PartSensor', type: 'digital-in', value: true },
  { id: 'di2', name: 'DI_SafetyGate', type: 'digital-in', value: true },
  { id: 'di3', name: 'DI_Emergency', type: 'digital-in', value: false },
  { id: 'ai1', name: 'AI_ForceSensor', type: 'analog-in', value: 42.7 },
  { id: 'ai2', name: 'AI_Temperature', type: 'analog-in', value: 23.1 },
  { id: 'ao1', name: 'AO_Speed', type: 'analog-out', value: 75 },
];

export const AppStateProvider = ({ children }: { children: ReactNode }) => {
  const [mode, setMode] = useState<AppMode>('edit');
  const [activePanel, setActivePanel] = useState<ActivePanel>('program');
  const [simState, setSimState] = useState<SimulationState>('stopped');
  const [selectedNodeId, setSelectedNodeId] = useState<string | null>('wp2');
  const [consoleEntries, setConsoleEntries] = useState<ConsoleEntry[]>([
    { id: 1, time: '10:24:01', type: 'info', message: 'RobotStudio v4.2.1 initialized' },
    { id: 2, time: '10:24:01', type: 'success', message: 'Connected to ROS 2 bridge' },
    { id: 3, time: '10:24:02', type: 'info', message: 'Robot model: IRB 6700 loaded' },
    { id: 4, time: '10:24:03', type: 'info', message: 'SolidWorks API connection established' },
    { id: 5, time: '10:24:05', type: 'warning', message: 'Joint 4 approaching soft limit (175.2°)' },
  ]);
  const [programTree, setProgramTree] = useState<ProgramNode[]>(defaultTree);
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
  const [consoleVisible, setConsoleVisible] = useState(true);
  const [codeEditorVisible, setCodeEditorVisible] = useState(true);
  const [settingsTab, setSettingsTab] = useState('graphics');
  const [editorMode, setEditorMode] = useState<'block' | 'script'>('script');

  let entryId = consoleEntries.length + 1;

  const addConsoleEntry = (type: ConsoleEntry['type'], message: string) => {
    const now = new Date();
    const time = now.toTimeString().slice(0, 8);
    setConsoleEntries(prev => [...prev, { id: entryId++, time, type, message }]);
  };

  const toggleTreeNode = (id: string) => {
    const toggle = (nodes: ProgramNode[]): ProgramNode[] =>
      nodes.map(n => n.id === id ? { ...n, expanded: !n.expanded } : { ...n, children: n.children ? toggle(n.children) : undefined });
    setProgramTree(toggle(programTree));
  };

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

  return (
    <AppContext.Provider value={{
      mode, setMode, activePanel, setActivePanel, simState, setSimState,
      selectedNodeId, setSelectedNodeId, consoleEntries, addConsoleEntry,
      programTree, setProgramTree, toggleTreeNode,
      waypoints, setWaypoints, ioPoints, setIOPoints, toggleIO,
      diagnostics, addDiagnostic, clearDiagnostics,
      showPaths, setShowPaths, showCollisions, setShowCollisions,
      simSpeed, setSimSpeed, breakpoints, toggleBreakpoint,
      projectName, setProjectName, robotType, setRobotType,
      rightPanelVisible, setRightPanelVisible,
      consoleVisible, setConsoleVisible,
      codeEditorVisible, setCodeEditorVisible,
      settingsTab, setSettingsTab, editorMode, setEditorMode,
    }}>
      {children}
    </AppContext.Provider>
  );
};
