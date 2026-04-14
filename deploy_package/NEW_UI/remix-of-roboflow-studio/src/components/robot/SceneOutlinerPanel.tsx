import React, { useState } from 'react';
import { useAppState, SceneObject } from '@/store/AppState';
import {
  ChevronDown, ChevronRight, Eye, EyeOff, Lock, Unlock,
  Box, Circle, Cylinder, Plus, Trash2, Copy, Focus,
  MapPin, Bot
} from 'lucide-react';

const ObjectIcon = ({ type }: { type: SceneObject['type'] }) => {
  switch (type) {
    case 'box': return <Box size={12} />;
    case 'cylinder': return <Cylinder size={12} />;
    case 'sphere': return <Circle size={12} />;
    default: return <Box size={12} />;
  }
};

const SceneObjectRow = ({ obj }: { obj: SceneObject }) => {
  const { selectedSceneObjectId, setSelectedSceneObjectId, updateSceneObject, removeSceneObject, setRightPanelTab, addConsoleEntry } = useAppState();
  const isSelected = selectedSceneObjectId === obj.id;
  const [contextMenu, setContextMenu] = useState<{ x: number; y: number } | null>(null);

  return (
    <>
      <div
        className={`flex items-center gap-1.5 px-2 py-1 cursor-pointer rounded-md transition-all duration-150 group ${
          isSelected ? 'bg-primary/12 text-primary' : 'hover:bg-accent/20 text-foreground'
        }`}
        onClick={() => { setSelectedSceneObjectId(obj.id); setRightPanelTab('object'); }}
        onContextMenu={e => {
          e.preventDefault();
          setContextMenu({ x: e.clientX, y: e.clientY });
        }}
      >
        <button className="shrink-0 opacity-50 hover:opacity-100 transition-opacity"
          onClick={e => { e.stopPropagation(); updateSceneObject(obj.id, { visible: !obj.visible }); }}>
          {obj.visible ? <Eye size={10} /> : <EyeOff size={10} className="text-muted-foreground/40" />}
        </button>
        <button className="shrink-0 opacity-50 hover:opacity-100 transition-opacity"
          onClick={e => { e.stopPropagation(); updateSceneObject(obj.id, { locked: !obj.locked }); }}>
          {obj.locked ? <Lock size={10} className="text-status-warning" /> : <Unlock size={10} />}
        </button>
        <ObjectIcon type={obj.type} />
        {isSelected ? (
          <input
            className="flex-1 bg-transparent text-[11px] font-medium focus:outline-none border-b border-primary/30"
            value={obj.name}
            autoFocus
            onChange={e => updateSceneObject(obj.id, { name: e.target.value })}
            onKeyDown={e => { if (e.key === 'Enter') setSelectedSceneObjectId(null); }}
          />
        ) : (
          <span className="text-[11px] truncate flex-1">{obj.name}</span>
        )}
        {obj.isCollision && <span className="text-[8px] px-1 rounded bg-status-warning/15 text-status-warning font-bold">COL</span>}
      </div>

      {contextMenu && (
        <>
          <div className="fixed inset-0 z-[100]" onClick={() => setContextMenu(null)} />
          <div className="fixed z-[101] w-48 py-1 rounded-xl glass-panel" style={{ left: contextMenu.x, top: contextMenu.y }}>
            <div className="px-3 py-1 text-[9px] text-muted-foreground/50 uppercase">{obj.name}</div>
            <button className="ctx-item" onClick={() => { setContextMenu(null); }}>
              <Focus size={12} /> Focus Camera
            </button>
            <button className="ctx-item" onClick={() => {
              const clone: SceneObject = { ...obj, id: `so-${Date.now()}`, name: obj.name + '_copy' };
              addConsoleEntry('info', `Duplicated ${obj.name}`);
              setContextMenu(null);
            }}>
              <Copy size={12} /> Duplicate
            </button>
            <div className="my-1 mx-2 h-px" style={{ background: 'hsl(0 0% 100% / 0.06)' }} />
            <button className="ctx-item" onClick={() => { updateSceneObject(obj.id, { isCollision: !obj.isCollision }); setContextMenu(null); }}>
              <Box size={12} /> {obj.isCollision ? 'Disable' : 'Enable'} Collision
            </button>
            <div className="my-1 mx-2 h-px" style={{ background: 'hsl(0 0% 100% / 0.06)' }} />
            <button className="ctx-item text-destructive" onClick={() => { removeSceneObject(obj.id); setContextMenu(null); addConsoleEntry('info', `Removed ${obj.name}`); }}>
              <Trash2 size={12} /> Delete
            </button>
          </div>
        </>
      )}
    </>
  );
};

export const SceneOutlinerPanel = () => {
  const { sceneObjects, addSceneObject, waypoints, robots } = useAppState();
  const [expanded, setExpanded] = useState({ robot: true, environment: true, objects: true, waypoints: true });

  const toggleSection = (key: keyof typeof expanded) => setExpanded(prev => ({ ...prev, [key]: !prev[key] }));

  const addPrimitive = (type: SceneObject['type']) => {
    const id = `so-${Date.now()}`;
    addSceneObject({
      id, name: `${type}_${sceneObjects.length + 1}`, type,
      position: { x: 0, y: 0, z: 800 }, rotation: { rx: 0, ry: 0, rz: 0 }, scale: { sx: 1, sy: 1, sz: 1 },
      visible: true, locked: false, isCollision: true,
      color: type === 'box' ? '#5A5A6A' : type === 'cylinder' ? '#4A6A5A' : '#6A4A5A',
      opacity: 1,
      width: 200, depth: 200, height: 200, radius: 100,
    });
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        Scene Outliner
        <div className="flex gap-1">
          <button className="glass-btn !px-1.5 !py-0.5" onClick={() => addPrimitive('box')} title="Add Box"><Box size={11} /></button>
          <button className="glass-btn !px-1.5 !py-0.5" onClick={() => addPrimitive('cylinder')} title="Add Cylinder"><Cylinder size={11} /></button>
          <button className="glass-btn !px-1.5 !py-0.5" onClick={() => addPrimitive('sphere')} title="Add Sphere"><Circle size={11} /></button>
        </div>
      </div>
      <div className="flex-1 overflow-auto py-1 px-1">
        {/* Robot */}
        <div className="mb-1">
          <button className="flex items-center gap-1 px-1 py-0.5 text-[10px] text-muted-foreground/70 font-bold uppercase tracking-widest w-full"
            onClick={() => toggleSection('robot')}>
            {expanded.robot ? <ChevronDown size={10} /> : <ChevronRight size={10} />}
            <Bot size={10} /> Robot
          </button>
          {expanded.robot && robots.map(r => (
            <div key={r.id} className="flex items-center gap-1.5 px-4 py-1 text-[11px] text-foreground">
              <div className="w-2 h-2 rounded-full bg-status-ok" />
              <span>{r.name}</span>
              <span className="text-[9px] text-muted-foreground ml-auto">{r.model}</span>
            </div>
          ))}
        </div>

        {/* Scene Objects */}
        <div className="mb-1">
          <button className="flex items-center gap-1 px-1 py-0.5 text-[10px] text-muted-foreground/70 font-bold uppercase tracking-widest w-full"
            onClick={() => toggleSection('objects')}>
            {expanded.objects ? <ChevronDown size={10} /> : <ChevronRight size={10} />}
            <Box size={10} /> Objects ({sceneObjects.length})
          </button>
          {expanded.objects && sceneObjects.map(obj => (
            <div key={obj.id} className="pl-2">
              <SceneObjectRow obj={obj} />
            </div>
          ))}
        </div>

        {/* Waypoints */}
        <div className="mb-1">
          <button className="flex items-center gap-1 px-1 py-0.5 text-[10px] text-muted-foreground/70 font-bold uppercase tracking-widest w-full"
            onClick={() => toggleSection('waypoints')}>
            {expanded.waypoints ? <ChevronDown size={10} /> : <ChevronRight size={10} />}
            <MapPin size={10} /> Waypoints ({waypoints.length})
          </button>
          {expanded.waypoints && waypoints.map(wp => (
            <div key={wp.id} className="flex items-center gap-1.5 px-4 py-1 text-[11px]">
              <div className={`w-2 h-2 rounded-full ${
                wp.wpType === 'approach' ? 'bg-status-info' : wp.wpType === 'pick' ? 'bg-status-ok' : wp.wpType === 'place' ? 'bg-status-warning' : 'bg-foreground/50'
              }`} />
              <span className="truncate">{wp.name}</span>
              <span className="text-[9px] text-muted-foreground ml-auto font-mono">({wp.x},{wp.y},{wp.z})</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
