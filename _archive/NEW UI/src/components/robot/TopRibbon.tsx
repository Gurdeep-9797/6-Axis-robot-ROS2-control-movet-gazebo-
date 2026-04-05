import React, { useState } from 'react';
import { useAppState } from '@/store/AppState';
import {
  File, FolderOpen, Save, Download, Scissors, Copy, Clipboard,
  Undo, Redo, Play, Pause, SkipForward, Square, Settings, Bug,
  Cpu, Radio, Wrench, LayoutGrid, ChevronDown
} from 'lucide-react';

const RibbonButton = ({ icon: Icon, label, onClick, disabled, active }: {
  icon: any; label: string; onClick?: () => void; disabled?: boolean; active?: boolean;
}) => (
  <button
    className={`ribbon-btn ${active ? 'active' : ''}`}
    onClick={onClick}
    disabled={disabled}
    title={label}
  >
    <Icon size={18} />
    <span className="leading-none">{label}</span>
  </button>
);

const RibbonSeparator = () => <div className="w-px h-10 bg-border mx-1" />;

export const TopRibbon = () => {
  const { mode, setMode, simState, setSimState, addConsoleEntry, setActivePanel } = useAppState();
  const [activeMenu, setActiveMenu] = useState<string | null>(null);

  const menuItems = [
    { label: 'File', items: ['New Project', 'Open Project', 'Save', 'Save As...', 'Export RAPID', 'Export URScript'] },
    { label: 'Edit', items: ['Undo', 'Redo', 'Cut', 'Copy', 'Paste', 'Find & Replace'] },
    { label: 'Simulation', items: ['Run', 'Pause', 'Stop', 'Step Forward', 'Reset', 'Speed Settings'] },
    { label: 'Deploy', items: ['Compile', 'Verify Program', 'Upload to Controller', 'Download from Controller'] },
    { label: 'View', items: ['Program Tree', 'Console', 'Properties', 'IO Panel', 'Diagnostics'] },
  ];

  const handleSimAction = (action: string) => {
    switch (action) {
      case 'run':
        setSimState('running');
        setMode('simulate');
        addConsoleEntry('info', '▶ Simulation started');
        break;
      case 'pause':
        setSimState('paused');
        addConsoleEntry('info', '⏸ Simulation paused');
        break;
      case 'stop':
        setSimState('stopped');
        addConsoleEntry('info', '⏹ Simulation stopped');
        break;
      case 'step':
        addConsoleEntry('info', '⏭ Step: Executing MoveJ WP_Approach');
        break;
    }
  };

  return (
    <div className="flex flex-col select-none" style={{ background: 'hsl(var(--ribbon-bg))' }}>
      {/* Menu bar */}
      <div className="flex items-center h-7 px-2 gap-0 text-xs border-b" style={{ borderColor: 'hsl(var(--ribbon-border))' }}>
        <div className="flex items-center gap-2 mr-4">
          <Cpu size={14} className="text-primary" />
          <span className="font-semibold text-primary">RoboForge</span>
        </div>
        {menuItems.map(menu => (
          <div key={menu.label} className="relative">
            <button
              className="px-3 py-1 hover:bg-accent rounded-sm transition-colors"
              onClick={() => setActiveMenu(activeMenu === menu.label ? null : menu.label)}
            >
              {menu.label}
            </button>
            {activeMenu === menu.label && (
              <>
                <div className="fixed inset-0 z-40" onClick={() => setActiveMenu(null)} />
                <div className="absolute top-full left-0 z-50 mt-0.5 w-52 py-1 rounded-sm border border-border shadow-xl" style={{ background: 'hsl(var(--popover))' }}>
                  {menu.items.map((item, i) => (
                    <button key={i} className="w-full text-left px-3 py-1.5 text-xs hover:bg-accent transition-colors"
                      onClick={() => {
                        setActiveMenu(null);
                        addConsoleEntry('info', `Menu: ${menu.label} → ${item}`);
                      }}>
                      {item}
                    </button>
                  ))}
                </div>
              </>
            )}
          </div>
        ))}
      </div>

      {/* Ribbon toolbar */}
      <div className="flex items-center h-16 px-2 gap-1 border-b" style={{ borderColor: 'hsl(var(--ribbon-border))' }}>
        {/* File group */}
        <div className="flex items-center gap-0.5">
          <RibbonButton icon={File} label="New" onClick={() => addConsoleEntry('info', 'Creating new project...')} />
          <RibbonButton icon={FolderOpen} label="Open" onClick={() => addConsoleEntry('info', 'Opening project...')} />
          <RibbonButton icon={Save} label="Save" onClick={() => addConsoleEntry('success', 'Project saved')} />
          <RibbonButton icon={Download} label="Export" onClick={() => addConsoleEntry('info', 'Exporting...')} />
        </div>
        <RibbonSeparator />

        {/* Edit group */}
        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Undo} label="Undo" />
          <RibbonButton icon={Redo} label="Redo" />
          <RibbonButton icon={Scissors} label="Cut" />
          <RibbonButton icon={Copy} label="Copy" />
          <RibbonButton icon={Clipboard} label="Paste" />
        </div>
        <RibbonSeparator />

        {/* Simulation group */}
        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Wrench} label="Compile" onClick={() => {
            addConsoleEntry('info', 'Compiling program...');
            setTimeout(() => addConsoleEntry('success', '✓ Compilation successful — 0 errors, 1 warning'), 800);
          }} />
          <RibbonButton icon={Play} label="Run" onClick={() => handleSimAction('run')} active={simState === 'running'} disabled={simState === 'running'} />
          <RibbonButton icon={Pause} label="Pause" onClick={() => handleSimAction('pause')} disabled={simState !== 'running'} />
          <RibbonButton icon={Square} label="Stop" onClick={() => handleSimAction('stop')} disabled={simState === 'stopped'} />
          <RibbonButton icon={SkipForward} label="Step" onClick={() => handleSimAction('step')} disabled={simState === 'running'} />
        </div>
        <RibbonSeparator />

        {/* Mode toggle */}
        <div className="flex flex-col items-center gap-1">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Mode</span>
          <div className="mode-toggle">
            {(['edit', 'simulate', 'live'] as const).map(m => (
              <button key={m} className={`mode-toggle-btn ${mode === m ? 'active' : ''}`}
                onClick={() => { setMode(m); addConsoleEntry('info', `Switched to ${m} mode`); }}>
                {m.charAt(0).toUpperCase() + m.slice(1)}
              </button>
            ))}
          </div>
        </div>
        <RibbonSeparator />

        {/* Quick access */}
        <div className="flex items-center gap-0.5">
          <RibbonButton icon={Bug} label="Debug" onClick={() => setActivePanel('diagnostics')} />
          <RibbonButton icon={Radio} label="I/O" onClick={() => setActivePanel('io')} />
          <RibbonButton icon={Settings} label="Settings" onClick={() => setActivePanel('settings')} />
        </div>

        {/* Spacer + status */}
        <div className="flex-1" />
        <div className="flex items-center gap-3 text-xs mr-2">
          <div className="flex items-center gap-1.5">
            <div className={`status-led ${simState === 'running' ? 'on animate-pulse-led' : 'off'}`} />
            <span className="text-muted-foreground">{simState === 'running' ? 'RUNNING' : simState === 'paused' ? 'PAUSED' : 'IDLE'}</span>
          </div>
          <div className="flex items-center gap-1.5">
            <div className="status-led on" />
            <span className="text-muted-foreground">ROS 2</span>
          </div>
          <div className="flex items-center gap-1.5">
            <div className="status-led on" />
            <span className="text-muted-foreground">DX12</span>
          </div>
        </div>
      </div>
    </div>
  );
};
