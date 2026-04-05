import React from 'react';
import { useAppState } from '@/store/AppState';
import { GripVertical, ArrowRight, RotateCw, Timer, GitBranch, Repeat } from 'lucide-react';

const blocks = [
  { id: 'movej', name: 'MoveJ', desc: 'Joint move', color: 'bg-block-move', icon: ArrowRight },
  { id: 'movel', name: 'MoveL', desc: 'Linear move', color: 'bg-block-move', icon: ArrowRight },
  { id: 'movec', name: 'MoveC', desc: 'Circular move', color: 'bg-block-move', icon: RotateCw },
  { id: 'wait', name: 'Wait', desc: 'Time delay', color: 'bg-block-wait', icon: Timer },
  { id: 'if', name: 'If/Else', desc: 'Conditional', color: 'bg-block-logic', icon: GitBranch },
  { id: 'while', name: 'While', desc: 'Loop', color: 'bg-block-loop', icon: Repeat },
  { id: 'setdo', name: 'SetDO', desc: 'Digital output', color: 'bg-block-io', icon: ArrowRight },
  { id: 'getdi', name: 'GetDI', desc: 'Digital input', color: 'bg-block-io', icon: ArrowRight },
];

export const BlockLibraryPanel = () => {
  const { addConsoleEntry } = useAppState();

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Block Library</div>
      <div className="p-2 space-y-1 overflow-auto flex-1">
        <p className="text-[10px] text-muted-foreground px-1 mb-2">Drag blocks to program editor</p>
        {blocks.map(block => (
          <div
            key={block.id}
            className={`block-item ${block.color} text-primary-foreground`}
            draggable
            onClick={() => addConsoleEntry('info', `Added ${block.name} block to program`)}
          >
            <GripVertical size={12} className="opacity-50" />
            <block.icon size={14} />
            <div className="flex flex-col">
              <span className="font-medium text-xs">{block.name}</span>
              <span className="text-[10px] opacity-70">{block.desc}</span>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
};
