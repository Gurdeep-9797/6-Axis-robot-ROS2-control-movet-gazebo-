import React from 'react';
import { useAppState } from '@/store/AppState';
import { Box, Trash2, Copy, Eye, EyeOff, Lock, Unlock } from 'lucide-react';

const SectionLabel = ({ children }: { children: React.ReactNode }) => (
  <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold mb-1.5">{children}</div>
);

const InputRow = ({ label, value, onChange, unit }: { label: string; value: number; onChange: (v: number) => void; unit?: string }) => (
  <div className="flex items-center justify-between py-0.5">
    <span className="text-xs text-muted-foreground font-mono">{label}</span>
    <div className="flex items-center gap-1">
      <input type="number" className="glass-input w-20 text-right tabular-nums" value={value}
        onChange={e => onChange(Number(e.target.value))} />
      {unit && <span className="text-[9px] text-muted-foreground w-8">{unit}</span>}
    </div>
  </div>
);

export const ObjectPropertiesPanel = () => {
  const { selectedSceneObjectId, sceneObjects, updateSceneObject, removeSceneObject, addConsoleEntry } = useAppState();
  const obj = sceneObjects.find(o => o.id === selectedSceneObjectId);

  if (!obj) {
    return (
      <div className="p-4 flex flex-col items-center justify-center h-full text-center">
        <Box size={20} className="text-muted-foreground mb-2" />
        <p className="text-xs text-muted-foreground">Select a scene object</p>
      </div>
    );
  }

  const update = (patch: Parameters<typeof updateSceneObject>[1]) => updateSceneObject(obj.id, patch);

  return (
    <div className="p-3 space-y-3 overflow-auto">
      {/* Header */}
      <div className="section-card">
        <div className="flex items-center gap-2 mb-2">
          <div className="w-4 h-4 rounded" style={{ background: obj.color }} />
          <input className="glass-input flex-1 text-xs font-bold" value={obj.name}
            onChange={e => update({ name: e.target.value })} />
        </div>
        <div className="flex gap-1">
          <button className={`glass-btn flex-1 text-[10px] py-1 ${obj.visible ? 'active' : ''}`}
            onClick={() => update({ visible: !obj.visible })}>
            {obj.visible ? <Eye size={10} className="inline mr-1" /> : <EyeOff size={10} className="inline mr-1" />}
            {obj.visible ? 'Visible' : 'Hidden'}
          </button>
          <button className={`glass-btn flex-1 text-[10px] py-1 ${obj.locked ? 'active' : ''}`}
            onClick={() => update({ locked: !obj.locked })}>
            {obj.locked ? <Lock size={10} className="inline mr-1" /> : <Unlock size={10} className="inline mr-1" />}
            {obj.locked ? 'Locked' : 'Free'}
          </button>
        </div>
      </div>

      {/* Transform */}
      <div>
        <SectionLabel>Position (mm)</SectionLabel>
        <InputRow label="X" value={obj.position.x} onChange={v => update({ position: { ...obj.position, x: v } })} unit="mm" />
        <InputRow label="Y" value={obj.position.y} onChange={v => update({ position: { ...obj.position, y: v } })} unit="mm" />
        <InputRow label="Z" value={obj.position.z} onChange={v => update({ position: { ...obj.position, z: v } })} unit="mm" />
      </div>

      <div>
        <SectionLabel>Rotation (°)</SectionLabel>
        <InputRow label="Rx" value={obj.rotation.rx} onChange={v => update({ rotation: { ...obj.rotation, rx: v } })} unit="°" />
        <InputRow label="Ry" value={obj.rotation.ry} onChange={v => update({ rotation: { ...obj.rotation, ry: v } })} unit="°" />
        <InputRow label="Rz" value={obj.rotation.rz} onChange={v => update({ rotation: { ...obj.rotation, rz: v } })} unit="°" />
      </div>

      {/* Dimensions */}
      <div>
        <SectionLabel>Dimensions</SectionLabel>
        {(obj.type === 'box' || obj.type === 'conveyor') && (
          <>
            <InputRow label="Width" value={obj.width || 200} onChange={v => update({ width: v })} unit="mm" />
            <InputRow label="Depth" value={obj.depth || 200} onChange={v => update({ depth: v })} unit="mm" />
            <InputRow label="Height" value={obj.height || 200} onChange={v => update({ height: v })} unit="mm" />
          </>
        )}
        {obj.type === 'cylinder' && (
          <>
            <InputRow label="Radius" value={obj.radius || 100} onChange={v => update({ radius: v })} unit="mm" />
            <InputRow label="Height" value={obj.height || 200} onChange={v => update({ height: v })} unit="mm" />
          </>
        )}
        {obj.type === 'sphere' && (
          <InputRow label="Radius" value={obj.radius || 100} onChange={v => update({ radius: v })} unit="mm" />
        )}
      </div>

      {/* Material */}
      <div>
        <SectionLabel>Material</SectionLabel>
        <div className="flex items-center justify-between py-1">
          <span className="text-xs text-muted-foreground">Color</span>
          <input type="color" value={obj.color} onChange={e => update({ color: e.target.value })} className="w-8 h-6 rounded cursor-pointer" />
        </div>
        <div className="flex items-center justify-between py-1">
          <span className="text-xs text-muted-foreground">Opacity</span>
          <input type="range" min={0} max={100} value={obj.opacity * 100}
            onChange={e => update({ opacity: Number(e.target.value) / 100 })}
            className="w-20 h-1 accent-primary" />
        </div>
      </div>

      {/* Collision */}
      <div className="section-card">
        <div className="flex items-center justify-between">
          <SectionLabel>Collision</SectionLabel>
          <button className={`glass-btn text-[10px] py-1 px-2 ${obj.isCollision ? 'active' : ''}`}
            onClick={() => update({ isCollision: !obj.isCollision })}>
            {obj.isCollision ? 'Enabled' : 'Disabled'}
          </button>
        </div>
      </div>

      {/* Actions */}
      <div className="flex gap-1">
        <button className="glass-btn flex-1 text-[10px] py-1.5" onClick={() => {
          addConsoleEntry('info', `Duplicated ${obj.name}`);
        }}>
          <Copy size={10} className="inline mr-1" /> Duplicate
        </button>
        <button className="glass-btn flex-1 text-[10px] py-1.5 text-destructive" onClick={() => {
          removeSceneObject(obj.id);
          addConsoleEntry('info', `Deleted ${obj.name}`);
        }}>
          <Trash2 size={10} className="inline mr-1" /> Delete
        </button>
      </div>
    </div>
  );
};
