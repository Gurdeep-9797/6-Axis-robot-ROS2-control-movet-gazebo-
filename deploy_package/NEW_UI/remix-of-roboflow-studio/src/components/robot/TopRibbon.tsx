import React, { useState, useCallback } from 'react';
import { useAppState, AppMode } from '@/store/AppState';
import {
  File, FolderOpen, Save, Download,
  Undo, Redo, Play, Pause, SkipForward, Square, Settings, Bug,
  Cpu, Radio, Wrench, FilePlus, Target, AlertTriangle, Zap, Gauge
} from 'lucide-react';
import { HealthCheckModal } from './HealthCheckModal';
import { useRobotStateStore } from '@/store/RosConnection';
import { backendConnector } from '@/services/BackendConnector';

const RibbonButton = ({ icon: Icon, label, onClick, disabled, active }: {
  icon: any; label: string; onClick?: () => void; disabled?: boolean; active?: boolean;
}) => (
  <button className={`ribbon-btn ${active ? 'active' : ''}`} onClick={onClick} disabled={disabled} title={label}>
    <Icon size={16} strokeWidth={1.5} />
    <span className="leading-none text-[10px]">{label}</span>
  </button>
);

const RibbonSeparator = () => <div className="w-px h-10 mx-1" style={{ background: 'hsl(0 0% 100% / 0.06)' }} />;

const STORAGE_KEY = 'roboforge-project';

export const TopRibbon = () => {
  const {
    mode, setMode, simState, setSimState, addConsoleEntry, setActivePanel,
    executionMode, setExecutionMode, setJointAngles, projectName, setProjectName,
    runProgram, programBlocks, setProgramBlocks, waypoints, setWaypoints,
    ioPoints, setIOPoints, setSelectedBlockId, globalStartPoint,
    ikMode, setIkMode,
  } = useAppState();

  const [activeMenu, setActiveMenu] = useState<string | null>(null);
  const [isHealthCheckOpen, setIsHealthCheckOpen] = useState(false);
  const [pendingMode, setPendingMode] = useState<AppMode | null>(null);
  const [showOfflineWarning, setShowOfflineWarning] = useState(false);
  const [pendingIkMode, setPendingIkMode] = useState<string | null>(null);
  const [motorSpeed, setMotorSpeed] = useState(50); // PWM duty cycle %
  const [connectionStatus, setConnectionStatus] = useState<'connected' | 'disconnected' | 'connecting'>('connecting');

  // Track actual backend connection status
  React.useEffect(() => {
    backendConnector.onStatus((status, connMode) => {
      setConnectionStatus(status);
    });
  }, []);

  const menuItems = [
    { label: 'File', items: ['New Project', 'Open Project', 'Save', 'Save As...', 'Export RAPID', 'Export URScript'] },
    { label: 'Edit', items: ['Undo', 'Redo', 'Cut', 'Copy', 'Paste', 'Find & Replace'] },
    { label: 'Simulation', items: ['Run', 'Pause', 'Stop', 'Step Forward', 'Reset', 'Speed Settings'] },
    { label: 'Deploy', items: ['Compile', 'Verify Program', 'Upload to Controller', 'Download from Controller'] },
    { label: 'View', items: ['Program Tree', 'Console', 'Properties', 'IO Panel', 'Diagnostics', 'ROS 2 Bridge'] },
  ];

  const handleNewProject = useCallback(() => {
    const name = 'Project_' + Date.now().toString(36);
    setProjectName(name);
    setProgramBlocks([{ id: 'main', name: 'Main Program', type: 'routine', expanded: true, children: [] }]);
    setJointAngles([0, 0, 0, 0, 0, 0]);
    setSelectedBlockId(null);
    setSimState('stopped');
    addConsoleEntry('success', `✓ New project "${name}" created`);
  }, [setProjectName, setProgramBlocks, setJointAngles, setSelectedBlockId, setSimState, addConsoleEntry]);

  const handleSave = useCallback(() => {
    const data = { projectName, programBlocks, waypoints, ioPoints, globalStartPoint, version: '8.0' };
    localStorage.setItem(STORAGE_KEY, JSON.stringify(data));
    addConsoleEntry('success', `✓ Project "${projectName}" saved`);
  }, [projectName, programBlocks, waypoints, ioPoints, globalStartPoint, addConsoleEntry]);

  const handleExport = useCallback(() => {
    const data = { projectName, programBlocks, waypoints, ioPoints, globalStartPoint, version: '8.0', exportedAt: new Date().toISOString() };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `${projectName.replace(/\s+/g, '_')}_${Date.now()}.rfp`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
    addConsoleEntry('success', `✓ Exported "${projectName}.rfp" to Downloads`);
  }, [projectName, programBlocks, waypoints, ioPoints, globalStartPoint, addConsoleEntry]);

  const handleOpen = useCallback(() => {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (!raw) return;
    const data = JSON.parse(raw);
    setProjectName(data.projectName);
    setProgramBlocks(data.programBlocks);
    setWaypoints(data.waypoints);
    setIOPoints(data.ioPoints);
    addConsoleEntry('success', `✓ Project "${data.projectName}" loaded`);
  }, [setProjectName, setProgramBlocks, setWaypoints, setIOPoints, addConsoleEntry]);

  const handleSimAction = (action: string) => {
    switch (action) {
      case 'run':
        runProgram();
        setMode('simulate');
        break;
      case 'pause':
        setSimState('paused');
        addConsoleEntry('info', '⏸ Paused');
        break;
      case 'stop':
        setSimState('stopped');
        addConsoleEntry('info', '⏹ Stopped');
        break;
      case 'step':
        addConsoleEntry('info', '⏭ Step forward');
        break;
      case 'compile':
        try {
          const payload = {
            metadata: {
              project: projectName,
              version: "1.0",
              timestamp: new Date().toISOString(),
              target_controller: "universal",
            },
            trajectory: programBlocks.map((b, i) => ({
              sequence: i,
              type: b.type,
              target_pose: b.type.startsWith('Move') ? { x: b.x, y: b.y, z: b.z, rx: b.rx, ry: b.ry, rz: b.rz } : null,
              parameters: { speed: b.speed, zone: b.zone }
            })).filter(cmd => cmd.type.startsWith('Move') || cmd.type === 'Wait' || cmd.type === 'SetDO')
          };
          const blob = new Blob([JSON.stringify(payload, null, 2)], { type: 'application/json' });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = `${projectName.replace(/\s+/g, '_')}_compiled.json`;
          document.body.appendChild(a);
          a.click();
          document.body.removeChild(a);
          URL.revokeObjectURL(url);
          addConsoleEntry('success', `✓ Successfully compiled program to universal JSON`);
        } catch (e) {
          addConsoleEntry('error', `✗ Compilation failed: ${e}`);
        }
        break;
    }
  };

  const handleModeSwitch = (m: AppMode) => {
    if (m === 'live') {
      setPendingMode('live');
      setIsHealthCheckOpen(true);
    } else {
      setMode(m);
      addConsoleEntry('info', `Mode: ${m}`);
    }
  };

  const onHealthCheckSuccess = () => {
    if (pendingMode) {
      setMode(pendingMode);
      addConsoleEntry('success', `✓ Pre-flight check passed. Mode: ${pendingMode.toUpperCase()}`);
      setPendingMode(null);
    }
    setIsHealthCheckOpen(false);
  };

  // ── IK Mode Change with Red Warning ───────────────────────────────
  const handleIkModeChange = (newMode: string) => {
    if (newMode === 'offline-analytical' || newMode === 'offline-numerical') {
      // Show red warning before switching to offline IK
      setPendingIkMode(newMode);
      setShowOfflineWarning(true);
    } else {
      setIkMode(newMode as any);
      addConsoleEntry('info', `IK Mode: MoveIt 2 (online)`);
    }
  };

  const confirmOfflineIk = () => {
    if (pendingIkMode) {
      setIkMode(pendingIkMode as any);
      addConsoleEntry('warning', `⚠️ OFFLINE IK ACTIVE — Using local JS solver, NO MoveIt collision checking!`);
      setShowOfflineWarning(false);
      setPendingIkMode(null);
    }
  };

  // ── Motor Speed/PWM Control ───────────────────────────────────────
  const handleMotorSpeedChange = (speed: number) => {
    setMotorSpeed(speed);
    // Publish to all joints
    for (let i = 0; i < 6; i++) {
      backendConnector.publishMotorConfig(i, {
        motor_type: 'DC',
        pid: { kp: speed * 0.02, ki: speed * 0.001 },
        gear_ratio: 100,
        max_current_a: speed * 0.1,
        signal_type: 'PWM',
      });
    }
    addConsoleEntry('info', `⚡ Motor speed: ${speed}% (PWM duty cycle)`);
  };

  const handleMenuAction = (menu: string, item: string) => {
    setActiveMenu(null);
    if (menu === 'File') {
      if (item === 'New Project') return handleNewProject();
      if (item === 'Open Project') return handleOpen();
      if (item === 'Save') return handleSave();
    }
    if (menu === 'Simulation') {
      if (item === 'Run') return handleSimAction('run');
      if (item === 'Pause') return handleSimAction('pause');
      if (item === 'Stop') return handleSimAction('stop');
      if (item === 'Reset') setJointAngles([0, 0, 0, 0, 0, 0]);
    }
    addConsoleEntry('info', `${menu} → ${item}`);
  };

  return (
    <div className="flex flex-col select-none" style={{ background: 'hsl(var(--panel-bg))', borderBottom: '1px solid hsl(0 0% 100% / 0.05)' }}>
      {/* Menu bar */}
      <div className="flex items-center h-7 px-2 gap-0 text-xs" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.04)' }}>
        <div className="flex items-center gap-2 mr-4">
          <Cpu size={13} className="text-primary" />
          <span className="font-bold text-primary text-[11px]">RoboForge</span>
          <span className="text-[9px] text-muted-foreground/50">v8.2</span>
        </div>
        {menuItems.map(menu => (
          <div key={menu.label} className="relative">
            <button className="px-2.5 py-1 hover:bg-accent/30 rounded-lg transition-colors text-[11px]"
              onClick={() => setActiveMenu(activeMenu === menu.label ? null : menu.label)}>
              {menu.label}
            </button>
            {activeMenu === menu.label && (
              <div className="absolute top-full left-0 z-50 mt-1 w-52 py-1 rounded-xl glass-panel shadow-2xl border border-glass-border">
                {menu.items.map((item, i) => (
                  <button key={i} className="w-full text-left px-3 py-1.5 text-[11px] hover:bg-primary/10 transition-colors rounded-lg"
                    onClick={() => handleMenuAction(menu.label, item)}>
                    {item}
                  </button>
                ))}
              </div>
            )}
          </div>
        ))}
        <div className="flex-1" />
        <span className="text-[10px] text-muted-foreground/60 font-mono">{projectName}</span>
      </div>

      {/* Ribbon toolbar */}
      <div className="flex items-center h-14 px-2 gap-1">
        <div className="flex items-center gap-0.5">
          <RibbonButton icon={FilePlus} label="New" onClick={handleNewProject} />
          <RibbonButton icon={FolderOpen} label="Open" onClick={handleOpen} />
          <RibbonButton icon={Save} label="Save" onClick={handleSave} />
          <RibbonButton icon={Download} label="Export" onClick={handleExport} />
        </div>
        <RibbonSeparator />

        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Target} label="Home" onClick={() => {
            setJointAngles(globalStartPoint);
            addConsoleEntry('info', `Moved to Global Start Point: ${globalStartPoint.map(j => j.toFixed(1)).join(', ')}°`);
          }} />
          <RibbonButton icon={Undo} label="Undo" />
          <RibbonButton icon={Redo} label="Redo" />
        </div>
        <RibbonSeparator />

        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Wrench} label="Compile" onClick={() => handleSimAction('compile')} />
          <RibbonButton icon={Play} label="Run" onClick={() => handleSimAction('run')} active={simState === 'running'} />
          <RibbonButton icon={Pause} label="Pause" onClick={() => handleSimAction('pause')} active={simState === 'paused'} />
          <RibbonButton icon={Square} label="Stop" onClick={() => handleSimAction('stop')} />
          <RibbonButton icon={SkipForward} label="Step" />
        </div>
        <RibbonSeparator />

        <div className="flex flex-col items-center gap-1">
          <span className="text-[8px] text-muted-foreground/60 uppercase tracking-widest text-center">Operation Mode</span>
          <div className="mode-toggle">
            {(['edit', 'simulate', 'live'] as const).map(m => (
              <button key={m} className={`mode-toggle-btn text-[10px] ${mode === m ? 'active shadow-lg' : ''}`}
                onClick={() => handleModeSwitch(m)}>
                {m.charAt(0).toUpperCase() + m.slice(1)}
              </button>
            ))}
          </div>
        </div>

        {/* IK Mode Indicator with Warning */}
        <div className="flex flex-col items-center gap-1">
          <span className="text-[8px] text-muted-foreground/60 uppercase tracking-widest text-center">IK Solver</span>
          <div className="flex items-center gap-1">
            {ikMode === 'moveit' ? (
              <button className="px-2 py-1 text-[9px] bg-green-500/20 text-green-400 rounded-md font-medium border border-green-500/30"
                onClick={() => handleIkModeChange('offline-numerical')}
                title="Click to switch to Offline IK">
                🟢 MoveIt 2
              </button>
            ) : (
              <button className="px-2 py-1 text-[9px] bg-red-500/30 text-red-400 rounded-md font-bold border border-red-500/50 animate-pulse"
                onClick={() => handleIkModeChange('moveit')}
                title="⚠️ OFFLINE — No collision checking! Click to restore MoveIt">
                ⚠️ OFFLINE
              </button>
            )}
          </div>
        </div>

        {/* Motor Speed / PWM Control */}
        <div className="flex flex-col items-center gap-1">
          <span className="text-[8px] text-muted-foreground/60 uppercase tracking-widest text-center flex items-center gap-0.5">
            <Gauge size={8} /> Motor PWM
          </span>
          <div className="flex items-center gap-1">
            <input type="range" min={5} max={100} value={motorSpeed}
              onChange={e => handleMotorSpeedChange(Number(e.target.value))}
              className="w-16 h-1 accent-primary cursor-pointer"
              title={`Motor speed: ${motorSpeed}% PWM`} />
            <span className="text-[9px] font-mono text-primary w-8 text-center">{motorSpeed}%</span>
          </div>
        </div>

        {/* Connection Status */}
        <div className="flex flex-col items-center gap-1">
          <span className="text-[8px] text-muted-foreground/60 uppercase tracking-widest text-center">Bridge</span>
          <div className={`px-2 py-1 text-[9px] rounded-md font-medium border ${connectionStatus === 'connected' ? 'bg-green-500/20 text-green-400 border-green-500/30' :
            connectionStatus === 'connecting' ? 'bg-yellow-500/20 text-yellow-400 border-yellow-500/30' :
              'bg-red-500/20 text-red-400 border-red-500/30'
            }`}>
            {connectionStatus === 'connected' ? '🟢 Live' : connectionStatus === 'connecting' ? '🟡 Connect...' : '🔴 Offline'}
          </div>
        </div>

        <HealthCheckModal
          isOpen={isHealthCheckOpen}
          onClose={() => { setIsHealthCheckOpen(false); setPendingMode(null); }}
          onSuccess={onHealthCheckSuccess}
        />

        {/* Offline IK Warning Modal */}
        {showOfflineWarning && (
          <div className="fixed inset-0 z-[9999] flex items-center justify-center bg-black/60 backdrop-blur-sm"
            onClick={() => setShowOfflineWarning(false)}>
            <div className="bg-red-950/95 border-2 border-red-500 rounded-2xl p-8 max-w-md shadow-2xl shadow-red-500/30"
              onClick={e => e.stopPropagation()}>
              <div className="flex items-center gap-3 mb-4">
                <AlertTriangle size={32} className="text-red-500 animate-pulse" />
                <h2 className="text-xl font-bold text-red-400">⚠️ Offline IK Warning</h2>
              </div>
              <div className="space-y-3 text-sm text-red-200/80">
                <p className="font-semibold text-red-300">You are switching to OFFLINE IK solver.</p>
                <ul className="list-disc list-inside space-y-1 text-red-200/70">
                  <li><strong>No MoveIt collision checking</strong> — robot may collide with obstacles</li>
                  <li><strong>No path planning</strong> — only point-to-point IK solutions</li>
                  <li><strong>Local JS solver only</strong> — analytical or numerical DLS</li>
                  <li><strong>NO motor commands sent</strong> — trajectories stay local</li>
                </ul>
                <p className="text-red-400/60 text-xs mt-4">
                  Use only for testing and development. For production, use MoveIt 2 with full collision checking.
                </p>
              </div>
              <div className="flex gap-3 mt-6">
                <button className="flex-1 px-4 py-2 bg-gray-700 hover:bg-gray-600 text-white rounded-lg text-sm"
                  onClick={() => setShowOfflineWarning(false)}>
                  Cancel
                </button>
                <button className="flex-1 px-4 py-2 bg-red-600 hover:bg-red-500 text-white rounded-lg text-sm font-bold"
                  onClick={confirmOfflineIk}>
                  ⚠️ Use Offline IK Anyway
                </button>
              </div>
            </div>
          </div>
        )}

        <RibbonSeparator />

        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Bug} label="Debug" onClick={() => setActivePanel('diagnostics')} />
          <RibbonButton icon={Radio} label="I/O" onClick={() => setActivePanel('io')} />
          <RibbonButton icon={Settings} label="Settings" onClick={() => setActivePanel('settings')} />
        </div>

        <div className="flex-1" />
        <div className="flex items-center gap-3 text-xs mr-2">
          <div className={`status-led ${simState === 'running' ? 'on animate-pulse' : 'off'}`} />
          <span className="text-[9px] text-muted-foreground uppercase">{simState}</span>
          <div className="status-led on" />
          <span className="text-[9px] text-muted-foreground">BRIDGE</span>
        </div>
      </div>
    </div>
  );
};
