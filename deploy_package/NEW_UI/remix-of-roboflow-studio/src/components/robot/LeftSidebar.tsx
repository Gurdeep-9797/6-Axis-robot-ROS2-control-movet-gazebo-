import React, { useState } from 'react';
import { useAppState, ActivePanel } from '@/store/AppState';
import {
  Code, Box, Move, Radio, Bug, Settings, FolderKanban, Wrench,
  ChevronLeft, ChevronRight, Wifi, Layers, Maximize2
} from 'lucide-react';

const panels: { id: ActivePanel; icon: any; label: string }[] = [
  { id: 'program', icon: Code, label: 'Program' },
  { id: 'blocks', icon: Box, label: 'Blocks' },
  { id: 'scene', icon: Layers, label: 'Scene' },
  { id: 'fullscreen3d', icon: Maximize2, label: '3D View' },
  { id: 'motion', icon: Move, label: 'Motion' },
  { id: 'io', icon: Radio, label: 'I/O' },
  { id: 'diagnostics', icon: Bug, label: 'Debug' },
  { id: 'config', icon: Wrench, label: 'Config' },
  { id: 'ros2', icon: Wifi, label: 'ROS 2' },
  { id: 'settings', icon: Settings, label: 'Settings' },
  { id: 'project', icon: FolderKanban, label: 'Project' },
];

export const LeftSidebar = () => {
  const { activePanel, setActivePanel } = useAppState();
  const [collapsed, setCollapsed] = useState(false);

  return (
    <div className="flex flex-col h-full" style={{
      width: collapsed ? 40 : 50,
      background: 'hsl(var(--panel-bg))',
      borderRight: '1px solid hsl(0 0% 100% / 0.05)',
    }}>
      <div className="flex flex-col items-center gap-0.5 py-2 flex-1">
        {panels.map(p => (
          <button
            key={p.id}
            title={p.label}
            className={`flex flex-col items-center justify-center w-10 h-10 rounded-xl transition-all duration-200
              ${activePanel === p.id
                ? p.id === 'fullscreen3d'
                  ? 'bg-cyan-500/15 text-cyan-400 shadow-[0_0_12px_hsl(180_80%_50%/0.15)]'
                  : 'bg-primary/12 text-primary shadow-[0_0_10px_hsl(var(--primary)/0.1)]'
                : 'text-muted-foreground hover:text-foreground hover:bg-accent/25'}`}
            onClick={() => setActivePanel(p.id)}
          >
            <p.icon size={16} strokeWidth={1.5} />
            {!collapsed && <span className="text-[8px] mt-0.5 leading-none font-medium">{p.label}</span>}
          </button>
        ))}
      </div>
      <button
        className="flex items-center justify-center h-8 text-muted-foreground/40 hover:text-foreground transition-colors"
        style={{ borderTop: '1px solid hsl(0 0% 100% / 0.04)' }}
        onClick={() => setCollapsed(!collapsed)}
      >
        {collapsed ? <ChevronRight size={12} /> : <ChevronLeft size={12} />}
      </button>
    </div>
  );
};
