import React, { useState } from 'react';
import { useAppState } from '@/store/AppState';
import { FolderOpen, FilePlus, Clock, Upload } from 'lucide-react';

const projects = [
  { id: '1', name: 'PickPlace_Cell_01', robot: 'IRB 6700', modified: '2026-03-20 09:45', status: 'active' },
  { id: '2', name: 'Welding_Line_03', robot: 'IRB 4600', modified: '2026-03-19 16:30', status: 'saved' },
  { id: '3', name: 'Assembly_Station_B', robot: 'UR10e', modified: '2026-03-18 11:20', status: 'saved' },
  { id: '4', name: 'Palletizing_Demo', robot: 'KUKA KR 210', modified: '2026-03-15 08:00', status: 'saved' },
];

export const ProjectManagerPanel = () => {
  const { addConsoleEntry } = useAppState();
  const [selected, setSelected] = useState('1');

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        Project Manager
        <button className="text-primary hover:text-primary/80 transition-colors" onClick={() => addConsoleEntry('info', 'Creating new project...')}>
          <FilePlus size={14} />
        </button>
      </div>
      <div className="flex-1 overflow-auto p-2 space-y-1">
        {projects.map(p => (
          <div key={p.id}
            className={`flex items-center gap-3 px-3 py-2.5 rounded-lg cursor-pointer transition-all duration-200 ${
              selected === p.id ? 'glass-surface shadow-[0_0_8px_hsl(var(--primary)/0.1)]' : 'hover:bg-accent/40'
            }`}
            onClick={() => { setSelected(p.id); addConsoleEntry('info', `Selected project: ${p.name}`); }}>
            <FolderOpen size={16} className={selected === p.id ? 'text-primary' : 'text-muted-foreground'} />
            <div className="flex-1 min-w-0">
              <div className="text-xs font-medium truncate">{p.name}</div>
              <div className="text-[10px] text-muted-foreground flex items-center gap-2">
                <span>{p.robot}</span>
                <span>•</span>
                <Clock size={10} />
                <span>{p.modified}</span>
              </div>
            </div>
            {p.status === 'active' && (
              <span className="text-[9px] px-1.5 py-0.5 rounded-full" style={{
                background: 'hsl(var(--status-ok) / 0.15)',
                color: 'hsl(var(--status-ok))',
              }}>Active</span>
            )}
          </div>
        ))}
      </div>
      <div className="p-2 flex gap-1" style={{ borderTop: '1px solid hsl(var(--glass-border))' }}>
        <button className="glass-btn flex-1 text-center" onClick={() => addConsoleEntry('info', 'Opening project...')}>
          <FolderOpen size={12} className="inline mr-1" /> Open
        </button>
        <button className="glass-btn flex-1 text-center" onClick={() => addConsoleEntry('info', 'Exporting...')}>
          <Upload size={12} className="inline mr-1" /> Export
        </button>
      </div>
    </div>
  );
};
