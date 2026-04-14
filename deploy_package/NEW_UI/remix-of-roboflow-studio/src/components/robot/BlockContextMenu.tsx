import React, { useEffect, useRef } from 'react';
import { useAppState, ProgramBlockNode } from '@/store/AppState';
import { createDefaultBlock } from './BlockLibraryPanel';
import { Copy, Trash2, ArrowUp, ArrowDown, Repeat, GitBranch, Plus, Play, Eye, Code2, MapPin, CircleDot, SkipForward, StepForward } from 'lucide-react';

interface Props {
  blockId: string;
  x: number;
  y: number;
  onClose: () => void;
}

export const BlockContextMenu = ({ blockId, x, y, onClose }: Props) => {
  const ref = useRef<HTMLDivElement>(null);
  const {
    programBlocks, setProgramBlocks, setSelectedBlockId, setRightPanelTab,
    addConsoleEntry, removeBlock, executeBlock, setEditingBlockId, updateBlock
  } = useAppState();

  useEffect(() => {
    const handler = (e: MouseEvent) => {
      if (ref.current && !ref.current.contains(e.target as Node)) onClose();
    };
    document.addEventListener('mousedown', handler);
    return () => document.removeEventListener('mousedown', handler);
  }, [onClose]);

  const findBlockAndParent = (
    nodes: ProgramBlockNode[], id: string, parent: ProgramBlockNode | null = null
  ): { block: ProgramBlockNode; parent: ProgramBlockNode | null; siblings: ProgramBlockNode[]; index: number } | null => {
    for (let i = 0; i < nodes.length; i++) {
      if (nodes[i].id === id) return { block: nodes[i], parent, siblings: nodes, index: i };
      if (nodes[i].children) {
        const found = findBlockAndParent(nodes[i].children!, id, nodes[i]);
        if (found) return found;
      }
    }
    return null;
  };

  const deepClone = (block: ProgramBlockNode): ProgramBlockNode => {
    const cloned: ProgramBlockNode = { ...block, id: `block-${Date.now()}-${Math.random().toString(36).slice(2, 6)}` };
    if (cloned.children) cloned.children = cloned.children.map(deepClone);
    if (cloned.elseChildren) cloned.elseChildren = cloned.elseChildren.map(deepClone);
    return cloned;
  };

  const mutateTree = (fn: (blocks: ProgramBlockNode[]) => ProgramBlockNode[]) => {
    setProgramBlocks(fn(JSON.parse(JSON.stringify(programBlocks))));
  };

  const info = findBlockAndParent(programBlocks, blockId);
  const block = info?.block;
  const hasChildren = info?.block.children !== undefined;
  const isMotion = block && ['MoveJ', 'MoveL', 'MoveC', 'MoveAbsJ', 'SearchL'].includes(block.type);

  const items = [
    // Header
    { type: 'header' as const, label: `${block?.type || ''}: ${block?.name || ''}` },
    'separator' as const,
    // Edit
    { label: 'Edit Parameters...', icon: Code2, action: () => { setEditingBlockId(blockId); onClose(); } },
    { label: 'Duplicate', icon: Copy, action: () => {
      mutateTree(blocks => {
        const found = findBlockAndParent(blocks, blockId);
        if (!found) return blocks;
        const clone = deepClone(found.block);
        clone.name = found.block.name + ' (copy)';
        found.siblings.splice(found.index + 1, 0, clone);
        return blocks;
      });
      addConsoleEntry('info', 'Block duplicated');
      onClose();
    }},
    { label: 'Move Up', icon: ArrowUp, action: () => {
      mutateTree(blocks => {
        const found = findBlockAndParent(blocks, blockId);
        if (!found || found.index === 0) return blocks;
        const [item] = found.siblings.splice(found.index, 1);
        found.siblings.splice(found.index - 1, 0, item);
        return blocks;
      });
      onClose();
    }, disabled: info?.index === 0 },
    { label: 'Move Down', icon: ArrowDown, action: () => {
      mutateTree(blocks => {
        const found = findBlockAndParent(blocks, blockId);
        if (!found || found.index >= found.siblings.length - 1) return blocks;
        const [item] = found.siblings.splice(found.index, 1);
        found.siblings.splice(found.index + 1, 0, item);
        return blocks;
      });
      onClose();
    }, disabled: info ? info.index >= info.siblings.length - 1 : true },
    'separator' as const,
    // Execute
    { type: 'submenu-header' as const, label: 'Execute' },
    { label: 'Run This Block', icon: Play, action: () => { executeBlock(blockId); onClose(); } },
    { label: 'Run From Here', icon: SkipForward, action: () => { addConsoleEntry('info', 'Run from here...'); onClose(); } },
    'separator' as const,
    // Wrap
    { label: 'Wrap in While', icon: Repeat, action: () => {
      mutateTree(blocks => {
        const found = findBlockAndParent(blocks, blockId);
        if (!found) return blocks;
        const wrapper = createDefaultBlock('While');
        wrapper.name = 'While TRUE';
        wrapper.children = [found.block];
        wrapper.expanded = true;
        found.siblings[found.index] = wrapper;
        return blocks;
      });
      addConsoleEntry('info', 'Wrapped in While loop');
      onClose();
    }},
    { label: 'Wrap in If', icon: GitBranch, action: () => {
      mutateTree(blocks => {
        const found = findBlockAndParent(blocks, blockId);
        if (!found) return blocks;
        const wrapper = createDefaultBlock('If');
        wrapper.name = 'If';
        wrapper.children = [found.block];
        wrapper.expanded = true;
        found.siblings[found.index] = wrapper;
        return blocks;
      });
      addConsoleEntry('info', 'Wrapped in If block');
      onClose();
    }},
    ...(isMotion ? [
      'separator' as const,
      { type: 'submenu-header' as const, label: 'Waypoint' },
      { label: 'Show in 3D Viewport', icon: Eye, action: () => { addConsoleEntry('info', 'Focusing camera on waypoint...'); onClose(); } },
      { label: 'Teach from Position', icon: MapPin, action: () => { addConsoleEntry('info', 'Teaching from current TCP position...'); onClose(); } },
    ] : []),
    'separator' as const,
    // Breakpoint
    { label: block?.hasBreakpoint ? 'Remove Breakpoint' : 'Toggle Breakpoint', icon: CircleDot, action: () => {
      updateBlock(blockId, { hasBreakpoint: !block?.hasBreakpoint });
      onClose();
    }},
    'separator' as const,
    // Code
    { type: 'submenu-header' as const, label: 'Code' },
    { label: 'View as RAPID', icon: Code2, action: () => { addConsoleEntry('info', 'View RAPID snippet...'); onClose(); } },
    ...(hasChildren ? [
      'separator' as const,
      { type: 'submenu-header' as const, label: 'Add Child' },
      { label: 'Add MoveJ', icon: Plus, action: () => {
        const newBlock = createDefaultBlock('MoveJ');
        mutateTree(blocks => {
          const found = findBlockAndParent(blocks, blockId);
          if (!found) return blocks;
          if (!found.block.children) found.block.children = [];
          found.block.children.push(newBlock);
          found.block.expanded = true;
          return blocks;
        });
        setSelectedBlockId(newBlock.id);
        setRightPanelTab('properties');
        onClose();
      }},
      { label: 'Add Wait', icon: Plus, action: () => {
        const newBlock = createDefaultBlock('Wait');
        mutateTree(blocks => {
          const found = findBlockAndParent(blocks, blockId);
          if (!found) return blocks;
          if (!found.block.children) found.block.children = [];
          found.block.children.push(newBlock);
          return blocks;
        });
        onClose();
      }},
    ] : []),
    'separator' as const,
    { label: 'Delete', icon: Trash2, action: () => { removeBlock(blockId); addConsoleEntry('info', 'Block deleted'); onClose(); }, danger: true },
  ];

  return (
    <>
      <div className="fixed inset-0 z-[100]" onClick={onClose} />
      <div ref={ref} className="fixed z-[101] w-56 py-1 rounded-xl glass-panel animate-slide-in-right max-h-[80vh] overflow-auto"
        style={{ left: Math.min(x, window.innerWidth - 240), top: Math.min(y, window.innerHeight - 400) }}>
        {items.map((item, i) => {
          if (item === 'separator') return <div key={i} className="my-0.5 mx-2 h-px" style={{ background: 'hsl(0 0% 100% / 0.06)' }} />;
          if (typeof item === 'object' && item.type === 'header') {
            return <div key={i} className="px-3 py-1 text-[9px] text-muted-foreground/50 font-bold truncate">{item.label}</div>;
          }
          if (typeof item === 'object' && item.type === 'submenu-header') {
            return <div key={i} className="px-3 py-0.5 text-[8px] text-muted-foreground/40 uppercase tracking-widest">{item.label}</div>;
          }
          const { label, icon: Icon, action, disabled, danger } = item as any;
          return (
            <button
              key={label + i}
              className={`w-full flex items-center gap-2 px-3 py-1.5 text-[11px] transition-colors rounded-lg mx-0.5 ${
                disabled ? 'opacity-30 pointer-events-none' : 'hover:bg-accent/30'
              } ${danger ? 'text-destructive' : ''}`}
              style={{ width: 'calc(100% - 4px)' }}
              onClick={action}
              disabled={disabled}
            >
              <Icon size={12} strokeWidth={1.5} />
              {label}
            </button>
          );
        })}
      </div>
    </>
  );
};
