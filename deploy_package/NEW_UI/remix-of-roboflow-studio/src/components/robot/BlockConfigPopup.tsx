import React, { useState } from 'react';
import { X, Eye } from 'lucide-react';
import { ZoneType } from '@/engine/MotionTypes';

export type BlockConfigType = 'MoveJ' | 'MoveL' | 'MoveC' | 'Wait' | 'If' | 'While' | 'SetDO' | 'GetDI' | 'Honing';

interface BlockConfig {
  type: BlockConfigType;
  // Motion params
  targetMode?: 'waypoint' | 'cartesian';
  x?: number; y?: number; z?: number;
  qw?: number; qx?: number; qy?: number; qz?: number;
  orientMode?: 'quaternion' | 'euler';
  rx?: number; ry?: number; rz?: number;
  speed?: number;
  accel?: number;
  zone?: ZoneType;
  // Wait
  duration?: number;
  // IO
  signal?: string;
  value?: boolean;
  // Logic
  condition?: string;
  // Circular
  viaX?: number; viaY?: number; viaZ?: number;
  // Honing specific
  boreDiameter?: number;
  strokeLength?: number;
  spindleRpm?: number;
  reciprocationSpeed?: number;
}

interface BlockConfigPopupProps {
  config: BlockConfig;
  onApply: (config: BlockConfig) => void;
  onClose: () => void;
  onPreview?: () => void;
}

const ZONE_OPTIONS: ZoneType[] = ['fine', 'z1', 'z5', 'z10', 'z50', 'z100'];

const FieldRow = ({ label, children }: { label: string; children: React.ReactNode }) => (
  <div className="flex items-center justify-between py-1.5">
    <span className="text-xs text-muted-foreground">{label}</span>
    <div className="flex items-center gap-1">{children}</div>
  </div>
);

const NumInput = ({ value, onChange, unit, width = 'w-20' }: {
  value: number; onChange: (v: number) => void; unit?: string; width?: string;
}) => (
  <div className="flex items-center gap-1">
    <input
      type="number"
      className={`property-value ${width} text-right`}
      value={value}
      onChange={e => onChange(Number(e.target.value))}
    />
    {unit && <span className="text-[10px] text-muted-foreground w-8">{unit}</span>}
  </div>
);

export const BlockConfigPopup: React.FC<BlockConfigPopupProps> = ({
  config: initialConfig,
  onApply,
  onClose,
  onPreview,
}) => {
  const [config, setConfig] = useState<BlockConfig>({ ...initialConfig });

  const update = (patch: Partial<BlockConfig>) => setConfig(prev => ({ ...prev, ...patch }));

  const isMotion = ['MoveJ', 'MoveL', 'MoveC'].includes(config.type);

  const typeColors: Record<string, string> = {
    MoveJ: 'hsl(var(--block-move))',
    MoveL: 'hsl(var(--block-move))',
    MoveC: 'hsl(var(--block-move))',
    Wait: 'hsl(var(--block-wait))',
    If: 'hsl(var(--block-logic))',
    While: 'hsl(var(--block-loop))',
    SetDO: 'hsl(var(--block-io))',
    GetDI: 'hsl(var(--block-io))',
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center" style={{ background: 'rgba(0,0,0,0.6)' }}>
      <div className="rounded border border-border shadow-2xl" style={{ background: 'hsl(var(--panel-bg))', width: 360 }}>
        {/* Header */}
        <div className="flex items-center justify-between px-4 py-2.5 border-b border-border" style={{ background: typeColors[config.type] || 'hsl(var(--panel-header))' }}>
          <span className="text-sm font-semibold text-primary-foreground">{config.type} Configuration</span>
          <button className="text-primary-foreground/70 hover:text-primary-foreground" onClick={onClose}><X size={16} /></button>
        </div>

        <div className="p-4 space-y-3 max-h-[70vh] overflow-auto">
          {/* MOTION BLOCKS */}
          {isMotion && (
            <>
              {/* Target mode */}
              <div>
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Target Mode</span>
                <div className="mode-toggle mt-1">
                  <button className={`mode-toggle-btn ${config.targetMode === 'waypoint' ? 'active' : ''}`}
                    onClick={() => update({ targetMode: 'waypoint' })}>Waypoint</button>
                  <button className={`mode-toggle-btn ${config.targetMode === 'cartesian' ? 'active' : ''}`}
                    onClick={() => update({ targetMode: 'cartesian' })}>Cartesian</button>
                </div>
              </div>

              {/* Position */}
              <div>
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Position</span>
                <div className="space-y-0.5 mt-1">
                  <FieldRow label="X"><NumInput value={config.x || 0} onChange={v => update({ x: v })} unit="mm" /></FieldRow>
                  <FieldRow label="Y"><NumInput value={config.y || 0} onChange={v => update({ y: v })} unit="mm" /></FieldRow>
                  <FieldRow label="Z"><NumInput value={config.z || 0} onChange={v => update({ z: v })} unit="mm" /></FieldRow>
                </div>
              </div>

              {/* Orientation */}
              <div>
                <div className="flex items-center justify-between">
                  <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Orientation</span>
                  <div className="mode-toggle">
                    <button className={`mode-toggle-btn text-[10px] ${config.orientMode === 'quaternion' ? 'active' : ''}`}
                      onClick={() => update({ orientMode: 'quaternion' })}>Quat</button>
                    <button className={`mode-toggle-btn text-[10px] ${config.orientMode === 'euler' ? 'active' : ''}`}
                      onClick={() => update({ orientMode: 'euler' })}>Euler</button>
                  </div>
                </div>
                {config.orientMode === 'euler' ? (
                  <div className="space-y-0.5 mt-1">
                    <FieldRow label="RX"><NumInput value={config.rx || 0} onChange={v => update({ rx: v })} unit="°" /></FieldRow>
                    <FieldRow label="RY"><NumInput value={config.ry || 0} onChange={v => update({ ry: v })} unit="°" /></FieldRow>
                    <FieldRow label="RZ"><NumInput value={config.rz || 0} onChange={v => update({ rz: v })} unit="°" /></FieldRow>
                  </div>
                ) : (
                  <div className="space-y-0.5 mt-1">
                    <FieldRow label="QW"><NumInput value={config.qw ?? 1} onChange={v => update({ qw: v })} width="w-16" /></FieldRow>
                    <FieldRow label="QX"><NumInput value={config.qx || 0} onChange={v => update({ qx: v })} width="w-16" /></FieldRow>
                    <FieldRow label="QY"><NumInput value={config.qy || 0} onChange={v => update({ qy: v })} width="w-16" /></FieldRow>
                    <FieldRow label="QZ"><NumInput value={config.qz || 0} onChange={v => update({ qz: v })} width="w-16" /></FieldRow>
                  </div>
                )}
              </div>

              {/* Circular via point */}
              {config.type === 'MoveC' && (
                <div>
                  <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Via Point</span>
                  <div className="space-y-0.5 mt-1">
                    <FieldRow label="VX"><NumInput value={config.viaX || 0} onChange={v => update({ viaX: v })} unit="mm" /></FieldRow>
                    <FieldRow label="VY"><NumInput value={config.viaY || 0} onChange={v => update({ viaY: v })} unit="mm" /></FieldRow>
                    <FieldRow label="VZ"><NumInput value={config.viaZ || 0} onChange={v => update({ viaZ: v })} unit="mm" /></FieldRow>
                  </div>
                </div>
              )}

              {/* Speed & Accel */}
              <div className="space-y-0.5">
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Dynamics</span>
                <FieldRow label="Speed"><NumInput value={config.speed || 200} onChange={v => update({ speed: v })} unit="mm/s" /></FieldRow>
                <FieldRow label="Accel"><NumInput value={config.accel || 100} onChange={v => update({ accel: v })} unit="mm/s²" /></FieldRow>
              </div>

              {/* Zone */}
              <div>
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Zone (Blending)</span>
                <div className="flex gap-1 mt-1 flex-wrap">
                  {ZONE_OPTIONS.map(z => (
                    <button
                      key={z}
                      className={`px-2 py-1 text-[10px] rounded border transition-colors ${
                        config.zone === z
                          ? 'bg-primary text-primary-foreground border-primary'
                          : 'bg-secondary text-muted-foreground border-border hover:bg-accent'
                      }`}
                      onClick={() => update({ zone: z })}
                    >
                      {z}
                    </button>
                  ))}
                </div>
              </div>
            </>
          )}

          {/* WAIT BLOCK */}
          {config.type === 'Wait' && (
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Duration</span>
              <FieldRow label="Time"><NumInput value={config.duration || 0.5} onChange={v => update({ duration: v })} unit="s" /></FieldRow>
            </div>
          )}

          {/* IO BLOCKS */}
          {(config.type === 'SetDO' || config.type === 'GetDI') && (
            <div className="space-y-2">
              <div>
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Signal</span>
                <input
                  className="property-value w-full mt-1"
                  value={config.signal || ''}
                  onChange={e => update({ signal: e.target.value })}
                  placeholder="DO_Gripper"
                />
              </div>
              {config.type === 'SetDO' && (
                <FieldRow label="Value">
                  <div className="mode-toggle">
                    <button className={`mode-toggle-btn ${config.value ? 'active' : ''}`} onClick={() => update({ value: true })}>ON</button>
                    <button className={`mode-toggle-btn ${!config.value ? 'active' : ''}`} onClick={() => update({ value: false })}>OFF</button>
                  </div>
                </FieldRow>
              )}
            </div>
          )}

          {/* LOGIC BLOCKS */}
          {(config.type === 'If' || config.type === 'While') && (
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Condition</span>
              <input
                className="property-value w-full mt-1 font-mono"
                value={config.condition || ''}
                onChange={e => update({ condition: e.target.value })}
                placeholder={config.type === 'While' ? 'TRUE' : 'DI_PartSensor = 1'}
              />
              {config.type === 'If' && (
                <p className="text-[10px] text-muted-foreground mt-1">Else branch will execute when condition is false</p>
              )}
            </div>
          )}

          {/* HONING BLOCKS */}
          {config.type === 'Honing' && (
            <div className="space-y-2">
              <div>
                <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-bold">Honing Parameters</span>
                <div className="space-y-0.5 mt-1 border-l-2 border-primary/40 pl-2">
                  <FieldRow label="Bore Diameter"><NumInput value={config.boreDiameter || 50} onChange={v => update({ boreDiameter: v })} unit="mm" /></FieldRow>
                  <FieldRow label="Stroke Length"><NumInput value={config.strokeLength || 100} onChange={v => update({ strokeLength: v })} unit="mm" /></FieldRow>
                  <FieldRow label="Spindle Speed"><NumInput value={config.spindleRpm || 300} onChange={v => update({ spindleRpm: v })} unit="RPM" /></FieldRow>
                  <FieldRow label="Reciprocation"><NumInput value={config.reciprocationSpeed || 15} onChange={v => update({ reciprocationSpeed: v })} unit="mm/s" /></FieldRow>
                </div>
              </div>
              <p className="text-[10px] text-muted-foreground italic">
                * Generates a precise helical/spiral path with dynamic velocity tracking.
              </p>
            </div>
          )}
        </div>

        {/* Footer */}
        <div className="flex items-center justify-between px-4 py-3 border-t border-border">
          {onPreview && (
            <button
              className="flex items-center gap-1 px-3 py-1.5 text-xs bg-secondary hover:bg-accent rounded transition-colors"
              onClick={onPreview}
            >
              <Eye size={12} /> Preview Path
            </button>
          )}
          <div className="flex items-center gap-2 ml-auto">
            <button className="px-3 py-1.5 text-xs text-muted-foreground hover:text-foreground transition-colors" onClick={onClose}>
              Cancel
            </button>
            <button
              className="px-4 py-1.5 text-xs bg-primary text-primary-foreground rounded hover:bg-primary/90 transition-colors font-medium"
              onClick={() => onApply(config)}
            >
              Apply
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};
