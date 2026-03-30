import React from 'react';
import { useAppState, ActivePanel } from '@/store/AppState';
import {
  Code, Box, Move, Radio, Bug, Settings, FolderKanban, Wrench,
  ChevronLeft, ChevronRight
} from 'lucide-react';
import { useState } from 'react';

const panels: { id: ActivePanel; icon: any; label: string }[] = [
  { id: 'program', icon: Code, label: 'Program' },
  { id: 'blocks', icon: Box, label: 'Blocks' },
  { id: 'motion', icon: Move, label: 'Motion' },
  { id: 'io', icon: Radio, label: 'I/O' },
  { id: 'diagnostics', icon: Bug, label: 'Debug' },
  { id: 'config', icon: Wrench, label: 'Config' },
  { id: 'settings', icon: Settings, label: 'Settings' },
  { id: 'project', icon: FolderKanban, label: 'Project' },
];

export const LeftSidebar = () => {
  const { activePanel, setActivePanel } = useAppState();
  const [collapsed, setCollapsed] = useState(false);

  return (
    <div className="flex flex-col border-r border-border h-full" style={{ background: 'hsl(var(--panel-bg))', width: collapsed ? 40 : 52 }}>
      <div className="flex flex-col items-center gap-0.5 py-2 flex-1">
        {panels.map(p => (
          <button
            key={p.id}
            title={p.label}
            className={`flex flex-col items-center justify-center w-10 h-10 rounded-sm transition-all duration-150
              ${activePanel === p.id ? 'bg-primary/15 text-primary' : 'text-muted-foreground hover:text-foreground hover:bg-accent'}`}
            onClick={() => setActivePanel(p.id)}
          >
            <p.icon size={18} />
            {!collapsed && <span className="text-[9px] mt-0.5 leading-none">{p.label}</span>}
          </button>
        ))}
      </div>
      <button
        className="flex items-center justify-center h-8 text-muted-foreground hover:text-foreground transition-colors border-t border-border"
        onClick={() => setCollapsed(!collapsed)}
      >
        {collapsed ? <ChevronRight size={14} /> : <ChevronLeft size={14} />}
      </button>
    </div>
  );
};
