import React from 'react';
import { useAppState, ProgramBlockNode } from '@/store/AppState';
import { ZoneType } from '@/engine/MotionTypes';
import { solveIK, jointsToDegrees, forwardKinematics } from '@/engine/IKSolver';
import { Crosshair, RotateCcw, Play, Trash2, Target, Save, AlertTriangle } from 'lucide-react';
import { JogPanel } from './JogPanel';
import { ObjectPropertiesPanel } from './ObjectPropertiesPanel';

const ZONE_OPTIONS: ZoneType[] = ['fine', 'z1', 'z5', 'z10', 'z50', 'z100'];

const SectionLabel = ({ children }: { children: React.ReactNode }) => (
  <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold mb-1.5 mt-2">{children}</div>
);

const Divider = () => (
  <div style={{ height: 1, background: 'hsl(0 0% 100% / 0.06)', margin: '8px 0' }} />
);

const GlassSelect = ({ value, options, onChange }: { value: string; options: { label: string; value: string }[]; onChange: (v: string) => void }) => (
  <select className="glass-input w-full" value={value} onChange={e => onChange(e.target.value)}>
    {options.map(o => <option key={o.value} value={o.value}>{o.label}</option>)}
  </select>
);

// Improved full-width input row
const InputRow = ({ label, value, onChange, unit, step, color }: { label: string; value: number; onChange: (v: number) => void; unit?: string; step?: number; color?: string }) => (
  <div className="flex items-center gap-2 py-0.5">
    <span className="text-[11px] font-mono w-6 shrink-0" style={{ color: color || 'hsl(var(--muted-foreground))' }}>{label}</span>
    <input type="number" step={step || 1} className="glass-input flex-1 text-right tabular-nums text-[12px]" value={value}
      onChange={e => onChange(Number(e.target.value))} />
    {unit && <span className="text-[9px] text-muted-foreground w-10 shrink-0 text-right">{unit}</span>}
  </div>
);

// Compact labeled input (for inline use)
const LabeledInput = ({ label, value, onChange, type = 'text', placeholder }: { label: string; value: string; onChange: (v: string) => void; type?: string; placeholder?: string }) => (
  <div className="flex items-center gap-2 py-0.5">
    <span className="text-[11px] text-muted-foreground w-16 shrink-0">{label}</span>
    <input type={type} className="glass-input flex-1 text-[12px]" value={value}
      placeholder={placeholder} onChange={e => onChange(e.target.value)} />
  </div>
);

// Workspace reachability check (IRB 6700 reach is ~2650mm)
const MAX_REACH_MM = 2650;
const isWithinWorkspace = (x: number, y: number, z: number): boolean => {
  const dist = Math.sqrt(x * x + y * y + (z > 0 ? (z - 500) ** 2 : z * z)); // approximate
  return dist <= MAX_REACH_MM && z >= -200;
};

const BlockProperties = () => {
  const { selectedBlockId, programBlocks, updateBlock, removeBlock, executeBlock, addConsoleEntry, ikMode, jointAngles, setJointAngles, pickingMode, setPickingMode, waypoints, setWaypoints, ioPoints, executingBlockId, setEditingBlockId } = useAppState();

  const findBlock = (nodes: ProgramBlockNode[]): ProgramBlockNode | null => {
    for (const n of nodes) {
      if (n.id === selectedBlockId) return n;
      if (n.children) { const found = findBlock(n.children); if (found) return found; }
    }
    return null;
  };

  const block = selectedBlockId ? findBlock(programBlocks) : null;

  if (!block) {
    return (
      <div className="p-4 flex flex-col items-center justify-center h-full text-center">
        <div className="w-12 h-12 rounded-2xl glass-surface flex items-center justify-center mb-3">
          <Crosshair size={20} className="text-muted-foreground" />
        </div>
        <p className="text-xs text-muted-foreground">Select a block to edit</p>
      </div>
    );
  }

  const update = (patch: Partial<ProgramBlockNode>) => updateBlock(block.id, patch);
  const isMotion = ['MoveJ', 'MoveL', 'MoveC', 'MoveAbsJ', 'SearchL'].includes(block.type);
  const isExecuting = executingBlockId === block.id;

  // Check if target is within workspace
  const withinWS = isMotion ? isWithinWorkspace(block.x ?? 0, block.y ?? 0, block.z ?? 0) : true;

  const handleSolveIK = () => {
    if (!block.x && !block.y && !block.z) return;
    const result = solveIK(
      { position: { x: block.x || 0, y: block.y || 0, z: block.z || 0 }, orientation: { qw: 1, qx: 0, qy: 0, qz: 0 } },
      ikMode, jointAngles
    );
    if (result.success) {
      setJointAngles(result.joints);
      addConsoleEntry('success', `IK solved — error: ${result.error.toFixed(2)}mm`);
    } else {
      addConsoleEntry('error', `IK failed — error: ${result.error.toFixed(2)}mm`);
    }
  };

  // Teach current position — captures the current joint angles as position
  const handleTeachPoint = () => {
    const fk = forwardKinematics(jointAngles);
    update({
      x: Math.round(fk.position.x),
      y: Math.round(fk.position.y),
      z: Math.round(fk.position.z),
      targetMode: 'manual',
    });
    addConsoleEntry('success', `📍 Taught point: [${Math.round(fk.position.x)}, ${Math.round(fk.position.y)}, ${Math.round(fk.position.z)}] mm`);
  };

  // Save current position as a named waypoint variable
  const handleSaveAsWaypoint = () => {
    const defaultName = `WP_${block.name.replace(/\s/g, '_')}`;
    const name = window.prompt('Waypoint name:', defaultName);
    if (!name) return; // user cancelled
    const fk = forwardKinematics(jointAngles);
    // Use block position if set, otherwise use FK of current joints
    const px = (block.x !== undefined && block.x !== 0) ? block.x : Math.round(fk.position.x);
    const py = (block.y !== undefined && block.y !== 0) ? block.y : Math.round(fk.position.y);
    const pz = (block.z !== undefined && block.z !== 0) ? block.z : Math.round(fk.position.z);
    const newWP = {
      id: `wp-${Date.now()}`,
      name,
      x: px, y: py, z: pz,
      speed: block.speed || 500,
      acceleration: block.accel || 100,
      type: block.type as 'MoveJ' | 'MoveL' | 'MoveC',
      zone: (block.zone || 'z50') as ZoneType,
      orientMode: 'quaternion' as const,
      qw: 1, qx: 0, qy: 0, qz: 0,
      rx: 0, ry: 90, rz: 0,
      wpType: 'approach' as const,
    };
    setWaypoints([...waypoints, newWP]);
    update({ waypointId: newWP.id, targetMode: 'waypoint' });
    addConsoleEntry('success', `💾 Saved waypoint: ${name} [${px}, ${py}, ${pz}]`);
  };

  return (
    <div className="p-3 space-y-1">
      {/* Block header */}
      <div className="section-card">
        <div className="flex items-center gap-2 mb-1.5">
          <div className={`w-3 h-3 rounded-full ${
            isMotion ? 'bg-status-info shadow-[0_0_6px_hsl(var(--status-info)/0.5)]' :
            block.type === 'Wait' ? 'bg-block-wait' :
            ['While', 'For'].includes(block.type) ? 'bg-block-loop' :
            ['SetDO', 'GetDI', 'PulseDO', 'WaitDI', 'SetAO', 'GetAI'].includes(block.type) ? 'bg-block-io' :
            ['GripperOpen', 'GripperClose'].includes(block.type) ? 'bg-status-info' : 'bg-status-ok'
          }`} />
          <span className="text-[12px] font-bold tracking-wide">{block.type}</span>
        </div>
        <input className="glass-input w-full text-[12px] mb-2" value={block.name} 
          onChange={e => update({ name: e.target.value })} placeholder="Block name" />
        <div className="flex gap-1">
          <button className="glass-btn-success flex-1 text-[10px] py-1.5" onClick={() => executeBlock(block.id)} disabled={isExecuting}>
            <Play size={10} className="inline mr-1" fill="currentColor" />{isExecuting ? 'Running...' : 'Run'}
          </button>
          <button className="glass-btn text-[10px] py-1.5 px-3" onClick={() => setEditingBlockId(block.id)}>Edit</button>
          <button className="glass-btn text-[10px] py-1.5 px-2 text-destructive" onClick={() => removeBlock(block.id)}>
            <Trash2 size={10} />
          </button>
        </div>
      </div>

      {/* MOTION BLOCKS */}
      {isMotion && (
        <>
          <SectionLabel>Target Mode</SectionLabel>
          <div className="grid grid-cols-4 gap-1">
            {(['waypoint', 'viewport', 'manual', 'joint'] as const).map(m => (
              <button key={m} className={`glass-btn text-[10px] py-1.5 px-1 ${block.targetMode === m ? 'active' : ''}`}
                onClick={() => {
                  update({ targetMode: m });
                  if (m === 'viewport') setPickingMode('waypoint');
                }}>
                {m === 'viewport' ? 'Pick' : m === 'waypoint' ? 'WP' : m === 'joint' ? 'Joint' : 'XYZ'}
              </button>
            ))}
          </div>

          {/* Teach Point & Save Waypoint buttons */}
          <div className="flex gap-1 mt-1">
            <button className="glass-btn-primary flex-1 text-[10px] py-1.5" onClick={handleTeachPoint}>
              <Target size={10} className="inline mr-1" />Teach Point
            </button>
            <button className="glass-btn flex-1 text-[10px] py-1.5" onClick={handleSaveAsWaypoint}>
              <Save size={10} className="inline mr-1" />Save as WP
            </button>
          </div>

          {block.targetMode === 'waypoint' && (
            <div>
              <SectionLabel>Waypoint</SectionLabel>
              <GlassSelect
                value={block.waypointId || ''}
                options={waypoints.map(w => ({ label: `${w.name} [${w.x},${w.y},${w.z}]`, value: w.id }))}
                onChange={v => {
                  const wp = waypoints.find(w => w.id === v);
                  if (wp) update({ waypointId: v, x: wp.x, y: wp.y, z: wp.z, speed: wp.speed, accel: wp.acceleration, zone: wp.zone });
                }}
              />
            </div>
          )}

          {block.targetMode === 'viewport' && (
            <div className="section-card text-center">
              <Crosshair size={18} className="text-primary mx-auto mb-1 animate-pulse" />
              <p className="text-[10px] text-primary font-medium">Click in viewport to pick</p>
              <button className="glass-btn-primary text-[10px] mt-2 w-full" onClick={() => setPickingMode('waypoint')}>Enter Pick Mode</button>
            </div>
          )}

          {block.targetMode === 'joint' && (
            <div>
              <SectionLabel>Joint Values (°)</SectionLabel>
              {(block.joints || [0, 0, 0, 0, 0, 0]).map((j, i) => (
                <InputRow key={i} label={`J${i + 1}`} value={Number((j * 180 / Math.PI).toFixed(1))} unit="°"
                  color={['#e74c3c', '#e67e22', '#2ecc71', '#3498db', '#9b59b6', '#1abc9c'][i]}
                  onChange={v => {
                    const newJoints = [...(block.joints || [0, 0, 0, 0, 0, 0])];
                    newJoints[i] = v * Math.PI / 180;
                    update({ joints: newJoints });
                  }} />
              ))}
            </div>
          )}

          {block.targetMode !== 'joint' && (
            <div>
              <SectionLabel>Position [mm]</SectionLabel>
              <InputRow label="X" value={block.x ?? 0} onChange={v => update({ x: v })} unit="mm" color="#e74c3c" />
              <InputRow label="Y" value={block.y ?? 0} onChange={v => update({ y: v })} unit="mm" color="#2ecc71" />
              <InputRow label="Z" value={block.z ?? 0} onChange={v => update({ z: v })} unit="mm" color="#3498db" />
              {!withinWS && (
                <div className="flex items-center gap-1 mt-1 px-2 py-1 rounded" style={{ background: 'hsl(var(--status-warning) / 0.15)' }}>
                  <AlertTriangle size={11} className="text-yellow-500 shrink-0" />
                  <span className="text-[9px] text-yellow-400">Outside workspace envelope ({MAX_REACH_MM}mm reach)</span>
                </div>
              )}
            </div>
          )}

          {block.targetMode !== 'joint' && (
            <div>
              <SectionLabel>Orientation [°]</SectionLabel>
              <InputRow label="Rx" value={block.rx ?? 0} onChange={v => update({ rx: v })} unit="°" color="#e74c3c" />
              <InputRow label="Ry" value={block.ry ?? 0} onChange={v => update({ ry: v })} unit="°" color="#2ecc71" />
              <InputRow label="Rz" value={block.rz ?? 0} onChange={v => update({ rz: v })} unit="°" color="#3498db" />
            </div>
          )}

          {block.type === 'MoveC' && (
            <div>
              <SectionLabel>Via Point [mm]</SectionLabel>
              <InputRow label="VX" value={block.viaX ?? 0} onChange={v => update({ viaX: v })} unit="mm" color="#e74c3c" />
              <InputRow label="VY" value={block.viaY ?? 0} onChange={v => update({ viaY: v })} unit="mm" color="#2ecc71" />
              <InputRow label="VZ" value={block.viaZ ?? 0} onChange={v => update({ viaZ: v })} unit="mm" color="#3498db" />
            </div>
          )}

          <Divider />

          <SectionLabel>Zone</SectionLabel>
          <div className="flex gap-1 flex-wrap">
            {ZONE_OPTIONS.map(z => (
              <button key={z} className={`glass-btn text-[10px] px-2.5 py-1.5 ${block.zone === z ? 'active' : ''}`}
                onClick={() => update({ zone: z })}>{z}</button>
            ))}
          </div>

          <SectionLabel>Dynamics</SectionLabel>
          <InputRow label="Speed" value={block.speed ?? 500} onChange={v => update({ speed: v })} unit="mm/s" />
          <InputRow label="Accel" value={block.accel ?? 100} onChange={v => update({ accel: v })} unit="mm/s²" />

          {block.type === 'SearchL' && (
            <>
              <SectionLabel>Search</SectionLabel>
              <LabeledInput label="Stop on" value={block.stopCondition || ''} onChange={v => update({ stopCondition: v })} />
              <InputRow label="Max Dist" value={block.maxDistance ?? 200} onChange={v => update({ maxDistance: v })} unit="mm" />
            </>
          )}

          <Divider />

          <div className="section-card">
            <div className="flex items-center justify-between mb-2">
              <SectionLabel>Joint Readout (°)</SectionLabel>
              <button className="glass-btn-primary text-[10px] px-2.5 py-1" onClick={handleSolveIK}>
                <RotateCcw size={9} className="inline mr-1" />Solve IK
              </button>
            </div>
            <div className="grid grid-cols-3 gap-x-4 gap-y-1">
              {jointsToDegrees(jointAngles).map((j, i) => (
                <div key={i} className="flex items-center justify-between">
                  <span className="text-[10px] font-mono" style={{ color: ['#e74c3c', '#e67e22', '#2ecc71', '#3498db', '#9b59b6', '#1abc9c'][i] }}>J{i + 1}</span>
                  <span className="text-[11px] font-mono tabular-nums">{j.toFixed(1)}°</span>
                </div>
              ))}
            </div>
          </div>
        </>
      )}

      {/* WAIT */}
      {block.type === 'Wait' && (
        <div>
          <SectionLabel>Duration</SectionLabel>
          <InputRow label="Time" value={block.duration ?? 0.5} onChange={v => update({ duration: v })} unit="s" step={0.1} />
        </div>
      )}

      {/* WHILE */}
      {block.type === 'While' && (
        <div className="space-y-2">
          <SectionLabel>Condition Type</SectionLabel>
          <div className="grid grid-cols-3 gap-1">
            {(['TRUE', 'DI', 'EXPR'] as const).map(t => (
              <button key={t} className={`glass-btn text-[10px] py-1.5 ${block.whileCondition?.type === t ? 'active' : ''}`}
                onClick={() => update({ whileCondition: { ...(block.whileCondition || { type: 'TRUE' }), type: t } })}>
                {t === 'TRUE' ? '∞ Loop' : t === 'DI' ? 'DI' : 'Expr'}
              </button>
            ))}
          </div>
          {block.whileCondition?.type === 'DI' && (
            <>
              <SectionLabel>DI Channel</SectionLabel>
              <GlassSelect value={String(block.whileCondition.channel || 0)}
                options={ioPoints.filter(io => io.type === 'digital-in').map((io, i) => ({ label: io.name, value: String(i) }))}
                onChange={v => update({ whileCondition: { ...block.whileCondition!, channel: Number(v) } })} />
              <div className="flex items-center justify-between">
                <span className="text-xs text-muted-foreground">State</span>
                <div className="mode-toggle">
                  <button className={`mode-toggle-btn text-[10px] ${block.whileCondition.state === 1 ? 'active' : ''}`}
                    onClick={() => update({ whileCondition: { ...block.whileCondition!, state: 1 } })}>HIGH</button>
                  <button className={`mode-toggle-btn text-[10px] ${block.whileCondition.state === 0 ? 'active' : ''}`}
                    onClick={() => update({ whileCondition: { ...block.whileCondition!, state: 0 } })}>LOW</button>
                </div>
              </div>
            </>
          )}
          {block.whileCondition?.type === 'EXPR' && (
            <div>
              <SectionLabel>Expression</SectionLabel>
              <input className="glass-input w-full font-mono text-[12px]" value={block.whileCondition.expr || ''}
                onChange={e => update({ whileCondition: { ...block.whileCondition!, expr: e.target.value } })}
                placeholder="counter < 10" />
            </div>
          )}
          <InputRow label="Max Iter" value={block.maxIterations ?? 0} onChange={v => update({ maxIterations: v })} />
          <p className="text-[10px] text-muted-foreground">{block.children?.length || 0} instructions in loop</p>
        </div>
      )}

      {/* FOR */}
      {block.type === 'For' && (
        <div className="space-y-2">
          <LabeledInput label="Variable" value={block.loopVar || 'i'} onChange={v => update({ loopVar: v })} />
          <InputRow label="From" value={block.loopFrom ?? 1} onChange={v => update({ loopFrom: v })} />
          <InputRow label="To" value={block.loopTo ?? 10} onChange={v => update({ loopTo: v })} />
          <InputRow label="Step" value={block.loopStep ?? 1} onChange={v => update({ loopStep: v })} />
        </div>
      )}

      {/* IF */}
      {block.type === 'If' && (
        <div className="space-y-2">
          <SectionLabel>Condition</SectionLabel>
          <input className="glass-input w-full font-mono text-[12px]" value={block.ifCondition || ''}
            onChange={e => update({ ifCondition: e.target.value })}
            placeholder="DI_PartSensor = 1" />
          <p className="text-[10px] text-muted-foreground">Then: {block.children?.length || 0} | Else: {block.elseChildren?.length || 0}</p>
        </div>
      )}

      {/* IO */}
      {(block.type === 'SetDO' || block.type === 'GetDI' || block.type === 'PulseDO' || block.type === 'WaitDI') && (
        <div className="space-y-2">
          <SectionLabel>Signal</SectionLabel>
          <GlassSelect value={block.signal || ''}
            options={ioPoints.filter(io => ['SetDO', 'PulseDO'].includes(block.type) ? io.type === 'digital-out' : io.type === 'digital-in')
              .map(io => ({ label: io.name, value: io.name }))}
            onChange={v => update({ signal: v })} />
          {block.type === 'SetDO' && (
            <div className="flex items-center justify-between">
              <span className="text-xs text-muted-foreground">Value</span>
              <div className="mode-toggle">
                <button className={`mode-toggle-btn text-[10px] ${block.value ? 'active' : ''}`} onClick={() => update({ value: true })}>ON</button>
                <button className={`mode-toggle-btn text-[10px] ${!block.value ? 'active' : ''}`} onClick={() => update({ value: false })}>OFF</button>
              </div>
            </div>
          )}
          {block.type === 'PulseDO' && (
            <>
              <InputRow label="Width" value={block.pulseWidth ?? 200} onChange={v => update({ pulseWidth: v })} unit="ms" />
              <InputRow label="Count" value={block.pulseCount ?? 1} onChange={v => update({ pulseCount: v })} />
            </>
          )}
          {block.type === 'WaitDI' && (
            <>
              <div className="flex items-center justify-between">
                <span className="text-xs text-muted-foreground">Expected</span>
                <div className="mode-toggle">
                  <button className={`mode-toggle-btn text-[10px] ${block.value ? 'active' : ''}`} onClick={() => update({ value: true })}>HIGH</button>
                  <button className={`mode-toggle-btn text-[10px] ${!block.value ? 'active' : ''}`} onClick={() => update({ value: false })}>LOW</button>
                </div>
              </div>
              <InputRow label="Timeout" value={block.timeout ?? 5000} onChange={v => update({ timeout: v })} unit="ms" />
            </>
          )}
        </div>
      )}

      {/* Analog IO */}
      {(block.type === 'SetAO' || block.type === 'GetAI') && (
        <div className="space-y-2">
          <SectionLabel>Signal</SectionLabel>
          <GlassSelect value={block.signal || ''}
            options={ioPoints.filter(io => block.type === 'SetAO' ? io.type === 'analog-out' : io.type === 'analog-in')
              .map(io => ({ label: io.name, value: io.name }))}
            onChange={v => update({ signal: v })} />
          {block.type === 'SetAO' && (
            <InputRow label="Value" value={block.analogValue ?? 0} onChange={v => update({ analogValue: v })} unit="V" step={0.1} />
          )}
        </div>
      )}

      {/* GRIPPER */}
      {(block.type === 'GripperOpen' || block.type === 'GripperClose') && (
        <div className="space-y-2">
          <SectionLabel>Gripper Parameters</SectionLabel>
          <InputRow label="Width" value={block.gripWidth ?? 80} onChange={v => update({ gripWidth: v })} unit="mm" />
          <InputRow label="Force" value={block.gripForce ?? 50} onChange={v => update({ gripForce: v })} unit="N" />
          <InputRow label="Speed" value={block.gripSpeed ?? 100} onChange={v => update({ gripSpeed: v })} unit="mm/s" />
          <LabeledInput label="Tool" value={block.toolRef || 'tool0'} onChange={v => update({ toolRef: v })} />
        </div>
      )}

      {/* VARIABLE */}
      {block.type === 'SetVar' && (
        <div className="space-y-2">
          <LabeledInput label="Name" value={block.varName || ''} onChange={v => update({ varName: v })} />
          <SectionLabel>Type</SectionLabel>
          <div className="flex gap-1 flex-wrap">
            {(['num', 'bool', 'string', 'pos'] as const).map(t => (
              <button key={t} className={`glass-btn text-[10px] px-3 py-1.5 ${block.varType === t ? 'active' : ''}`}
                onClick={() => update({ varType: t })}>{t}</button>
            ))}
          </div>
          <LabeledInput label="Value" value={block.varValue || ''} onChange={v => update({ varValue: v })} />
        </div>
      )}

      {/* INCREMENT */}
      {block.type === 'Increment' && (
        <div className="space-y-2">
          <LabeledInput label="Variable" value={block.varName || ''} onChange={v => update({ varName: v })} />
          <InputRow label="Amount" value={block.incrementAmount ?? 1} onChange={v => update({ incrementAmount: v })} />
        </div>
      )}

      {/* CALL PROC */}
      {block.type === 'CallProc' && (
        <LabeledInput label="Procedure" value={block.procName || ''} onChange={v => update({ procName: v })} />
      )}

      {/* STOP */}
      {block.type === 'StopProgram' && (
        <LabeledInput label="Reason" value={block.reason || ''} onChange={v => update({ reason: v })} />
      )}

      {/* TOOL CHANGE */}
      {block.type === 'ToolChange' && (
        <LabeledInput label="New Tool" value={block.toolRef || ''} onChange={v => update({ toolRef: v })} />
      )}

      {/* ROUTINE */}
      {(block.type === 'routine' || block.type === 'Module') && (
        <div className="space-y-2">
          <SectionLabel>{block.type === 'Module' ? 'Module Name' : 'Routine Name'}</SectionLabel>
          <input className="glass-input w-full text-[12px]" value={block.name} onChange={e => update({ name: e.target.value })} />
          <p className="text-[10px] text-muted-foreground">{block.children?.length || 0} instructions</p>
        </div>
      )}
    </div>
  );
};

const WaypointProperties = () => {
  const { selectedNodeId, waypoints, setWaypoints, ikMode, jointAngles, setJointAngles, addConsoleEntry, selectedBlockId } = useAppState();

  if (selectedBlockId) return <BlockProperties />;

  const selectedWaypoint = waypoints.find(w => w.id === selectedNodeId);
  if (!selectedWaypoint) return <BlockProperties />;

  const updateWP = (field: string, value: any) => {
    setWaypoints(waypoints.map(w => w.id === selectedWaypoint.id ? { ...w, [field]: value } : w));
  };

  const withinWS = isWithinWorkspace(selectedWaypoint.x, selectedWaypoint.y, selectedWaypoint.z);

  const handleSolveIK = () => {
    const result = solveIK(
      { position: { x: selectedWaypoint.x, y: selectedWaypoint.y, z: selectedWaypoint.z }, orientation: { qw: 1, qx: 0, qy: 0, qz: 0 } },
      ikMode, jointAngles
    );
    if (result.success) {
      setJointAngles(result.joints);
      addConsoleEntry('success', `IK solved — error: ${result.error.toFixed(2)}mm`);
    } else {
      addConsoleEntry('error', `IK failed — error: ${result.error.toFixed(2)}mm`);
    }
  };

  // Teach current robot position as this waypoint
  const handleTeachWP = () => {
    const fk = forwardKinematics(jointAngles);
    updateWP('x', Math.round(fk.position.x));
    updateWP('y', Math.round(fk.position.y));
    updateWP('z', Math.round(fk.position.z));
    addConsoleEntry('success', `📍 Taught waypoint: ${selectedWaypoint.name} → [${Math.round(fk.position.x)}, ${Math.round(fk.position.y)}, ${Math.round(fk.position.z)}]`);
  };

  return (
    <div className="p-3 space-y-1">
      <div className="section-card">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${
            selectedWaypoint.wpType === 'approach' ? 'bg-status-info' :
            selectedWaypoint.wpType === 'pick' ? 'bg-status-ok' :
            selectedWaypoint.wpType === 'place' ? 'bg-status-warning' : 'bg-foreground/50'
          } shadow-[0_0_6px_currentColor]`} />
          <input className="glass-input flex-1 text-[12px] font-bold" value={selectedWaypoint.name}
            onChange={e => updateWP('name', e.target.value)} />
        </div>
      </div>

      <div className="flex gap-1">
        <button className="glass-btn-primary flex-1 text-[10px] py-1.5" onClick={handleTeachWP}>
          <Target size={10} className="inline mr-1" />Teach Current
        </button>
      </div>

      <SectionLabel>Motion Type</SectionLabel>
      <div className="mode-toggle">
        {(['MoveJ', 'MoveL', 'MoveC'] as const).map(t => (
          <button key={t} className={`mode-toggle-btn ${selectedWaypoint.type === t ? 'active' : ''}`}
            onClick={() => updateWP('type', t)}>{t}</button>
        ))}
      </div>

      <SectionLabel>Position [mm]</SectionLabel>
      <InputRow label="X" value={selectedWaypoint.x} onChange={v => updateWP('x', v)} unit="mm" color="#e74c3c" />
      <InputRow label="Y" value={selectedWaypoint.y} onChange={v => updateWP('y', v)} unit="mm" color="#2ecc71" />
      <InputRow label="Z" value={selectedWaypoint.z} onChange={v => updateWP('z', v)} unit="mm" color="#3498db" />

      {!withinWS && (
        <div className="flex items-center gap-1 px-2 py-1 rounded" style={{ background: 'hsl(var(--status-warning) / 0.15)' }}>
          <AlertTriangle size={11} className="text-yellow-500 shrink-0" />
          <span className="text-[9px] text-yellow-400">Point outside workspace envelope</span>
        </div>
      )}

      <Divider />

      <SectionLabel>Zone</SectionLabel>
      <div className="flex gap-1 flex-wrap">
        {ZONE_OPTIONS.map(z => (
          <button key={z} className={`glass-btn text-[10px] px-2.5 py-1.5 ${selectedWaypoint.zone === z ? 'active' : ''}`}
            onClick={() => updateWP('zone', z)}>{z}</button>
        ))}
      </div>

      <SectionLabel>Dynamics</SectionLabel>
      <InputRow label="Speed" value={selectedWaypoint.speed} onChange={v => updateWP('speed', v)} unit="mm/s" />
      <InputRow label="Accel" value={selectedWaypoint.acceleration} onChange={v => updateWP('acceleration', v)} unit="mm/s²" />

      <Divider />

      <div className="section-card">
        <div className="flex items-center justify-between mb-2">
          <SectionLabel>Joint Values (°)</SectionLabel>
          <button className="glass-btn-primary text-[10px] px-2.5 py-1" onClick={handleSolveIK}>Solve IK</button>
        </div>
        <div className="grid grid-cols-3 gap-x-4 gap-y-1">
          {jointsToDegrees(jointAngles).map((j, i) => (
            <div key={i} className="flex items-center justify-between">
              <span className="text-[10px] font-mono" style={{ color: ['#e74c3c', '#e67e22', '#2ecc71', '#3498db', '#9b59b6', '#1abc9c'][i] }}>J{i + 1}</span>
              <span className="text-[11px] font-mono tabular-nums">{j.toFixed(1)}°</span>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};

export const PropertiesPanel = () => {
  const { rightPanelTab, setRightPanelTab } = useAppState();

  return (
    <div className="flex flex-col h-full">
      <div className="flex" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.05)' }}>
        {(['properties', 'jog', 'object'] as const).map(tab => (
          <button key={tab} className={`flex-1 px-2 py-2.5 text-[10px] font-semibold uppercase tracking-widest transition-all duration-200 ${
            rightPanelTab === tab
              ? 'text-primary border-b-2 border-primary bg-primary/5'
              : 'text-muted-foreground hover:text-foreground hover:bg-accent/20'
          }`} onClick={() => setRightPanelTab(tab)}>
            {tab === 'properties' ? 'Props' : tab === 'jog' ? 'JOG' : 'Object'}
          </button>
        ))}
      </div>
      <div className="flex-1 overflow-auto">
        {rightPanelTab === 'properties' ? <WaypointProperties /> :
         rightPanelTab === 'jog' ? <JogPanel /> : <ObjectPropertiesPanel />}
      </div>
    </div>
  );
};
