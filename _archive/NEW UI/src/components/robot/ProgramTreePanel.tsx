import React from 'react';
import { useAppState, ProgramNode } from '@/store/AppState';
import { ChevronRight, ChevronDown, Folder, FileCode, MapPin, Zap } from 'lucide-react';

const nodeIcon = (type: ProgramNode['type']) => {
  switch (type) {
    case 'folder': return Folder;
    case 'routine': return FileCode;
    case 'waypoint': return MapPin;
    case 'instruction': return Zap;
  }
};

const TreeItem = ({ node, depth }: { node: ProgramNode; depth: number }) => {
  const { selectedNodeId, setSelectedNodeId, toggleTreeNode } = useAppState();
  const Icon = nodeIcon(node.type);
  const hasChildren = node.children && node.children.length > 0;

  return (
    <div>
      <div
        className={`tree-node ${selectedNodeId === node.id ? 'selected' : ''}`}
        style={{ paddingLeft: depth * 16 + 8 }}
        onClick={() => {
          setSelectedNodeId(node.id);
          if (hasChildren) toggleTreeNode(node.id);
        }}
      >
        {hasChildren ? (
          node.expanded ? <ChevronDown size={12} className="text-muted-foreground shrink-0" /> : <ChevronRight size={12} className="text-muted-foreground shrink-0" />
        ) : <span className="w-3 shrink-0" />}
        <Icon size={14} className={node.type === 'waypoint' ? 'text-status-info' : node.type === 'routine' ? 'text-status-ok' : 'text-muted-foreground'} />
        <span className="truncate">{node.name}</span>
      </div>
      {hasChildren && node.expanded && node.children!.map(child => (
        <TreeItem key={child.id} node={child} depth={depth + 1} />
      ))}
    </div>
  );
};

export const ProgramTreePanel = () => {
  const { programTree } = useAppState();

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Program Tree</div>
      <div className="flex-1 overflow-auto py-1">
        {programTree.map(node => (
          <TreeItem key={node.id} node={node} depth={0} />
        ))}
      </div>
    </div>
  );
};
