import React, { useCallback } from 'react';
import { useAppState, ProgramBlockNode, BlockNodeType } from '@/store/AppState';
import { GripVertical } from 'lucide-react';

export interface BlockDef {
  type: BlockNodeType;
  name: string;
  desc: string;
  badge: string;
  bg: string;
  fg: string;
  category: string;
}

export const BLOCK_DEFS: BlockDef[] = [
  // Motion
  { type: 'MoveJ', name: 'MoveJ', desc: 'Joint space move', badge: 'J', bg: '#185FA5', fg: '#B5D4F4', category: 'Motion' },
  { type: 'MoveL', name: 'MoveL', desc: 'Linear TCP move', badge: 'L', bg: '#3B6D11', fg: '#C0DD97', category: 'Motion' },
  { type: 'MoveC', name: 'MoveC', desc: 'Circular arc move', badge: 'C', bg: '#085041', fg: '#9FE1CB', category: 'Motion' },
  { type: 'MoveAbsJ', name: 'MoveAbsJ', desc: 'Absolute joint pos', badge: 'AJ', bg: '#0D3B6F', fg: '#8CB8E0', category: 'Motion' },
  { type: 'SearchL', name: 'SearchL', desc: 'Linear until trigger', badge: 'SL', bg: '#8B4513', fg: '#F5D6B8', category: 'Motion' },
  { type: 'Honing', name: 'Honing Cycle', desc: 'Helical/Spiral honing path', badge: 'H', bg: '#A32D2D', fg: '#F7C1C1', category: 'Motion' },
  // IO
  { type: 'SetDO', name: 'SetDO', desc: 'Digital output', badge: '⚡', bg: '#854F0B', fg: '#FAC775', category: 'I/O' },
  { type: 'PulseDO', name: 'PulseDO', desc: 'Pulse output', badge: '⚡⚡', bg: '#9B6B0A', fg: '#FFD700', category: 'I/O' },
  { type: 'GetDI', name: 'GetDI', desc: 'Digital input', badge: '◉', bg: '#993C1D', fg: '#F5C4B3', category: 'I/O' },
  { type: 'WaitDI', name: 'WaitDI', desc: 'Wait for DI', badge: 'DI', bg: '#6B2FA0', fg: '#D4B0F0', category: 'I/O' },
  { type: 'SetAO', name: 'SetAO', desc: 'Analog output', badge: 'AO', bg: '#0B7B8A', fg: '#A0E8F0', category: 'I/O' },
  { type: 'GetAI', name: 'GetAI', desc: 'Analog input', badge: 'AI', bg: '#0B6A8A', fg: '#90D8F0', category: 'I/O' },
  // Flow
  { type: 'Wait', name: 'Wait', desc: 'Time delay', badge: 'W', bg: '#3C3489', fg: '#CECBF6', category: 'Flow' },
  { type: 'If', name: 'If / Else', desc: 'Conditional branch', badge: '?', bg: '#72243E', fg: '#F4C0D1', category: 'Flow' },
  { type: 'While', name: 'While', desc: 'Conditional loop', badge: '↺', bg: '#A32D2D', fg: '#F7C1C1', category: 'Flow' },
  { type: 'For', name: 'For', desc: 'Count loop', badge: 'F', bg: '#8B2020', fg: '#F5AAAA', category: 'Flow' },
  { type: 'Break', name: 'Break', desc: 'Exit loop', badge: '⤴', bg: '#6B1E1E', fg: '#E8A0A0', category: 'Flow' },
  { type: 'Return', name: 'Return', desc: 'Exit routine', badge: '⏎', bg: '#555', fg: '#CCC', category: 'Flow' },
  // Robot
  { type: 'GripperOpen', name: 'Gripper Open', desc: 'Open gripper', badge: '⬌', bg: '#1A6B8A', fg: '#A0D8F0', category: 'Robot' },
  { type: 'GripperClose', name: 'Gripper Close', desc: 'Close gripper', badge: '⬄', bg: '#1A6B8A', fg: '#A0D8F0', category: 'Robot' },
  { type: 'ToolChange', name: 'Tool Change', desc: 'Change tool', badge: 'T', bg: '#0B7B8A', fg: '#A0E8F0', category: 'Robot' },
  { type: 'ResetError', name: 'Reset Error', desc: 'Clear error', badge: '✓', bg: '#2D5A3D', fg: '#A8E6CF', category: 'Robot' },
  { type: 'StopProgram', name: 'Stop', desc: 'Halt execution', badge: '■', bg: '#A32D2D', fg: '#F7C1C1', category: 'Robot' },
  // Structure
  { type: 'routine', name: 'Routine', desc: 'Procedure group', badge: 'fn', bg: '#2D5A3D', fg: '#A8E6CF', category: 'Structure' },
  { type: 'CallProc', name: 'Call Proc', desc: 'Call procedure', badge: '→fn', bg: '#2D5A3D', fg: '#A8E6CF', category: 'Structure' },
  // Variable
  { type: 'SetVar', name: 'SetVar', desc: 'Assign variable', badge: '=', bg: '#555', fg: '#CCC', category: 'Variable' },
  { type: 'Increment', name: 'Increment', desc: 'Add to variable', badge: '+1', bg: '#555', fg: '#CCC', category: 'Variable' },
];

export const BADGE_CONFIG: Record<string, { badge: string; bg: string; fg: string }> = Object.fromEntries(
  BLOCK_DEFS.map(d => [d.type, { badge: d.badge, bg: d.bg, fg: d.fg }])
);

export function createDefaultBlock(type: BlockNodeType): ProgramBlockNode {
  const id = `block-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`;
  const base: ProgramBlockNode = { id, name: `${type} (new)`, type };

  switch (type) {
    case 'MoveJ': case 'MoveL':
      return { ...base, targetMode: 'manual', x: 0, y: 0, z: 200, speed: 500, accel: 100, zone: 'z50' };
    case 'MoveC':
      return { ...base, targetMode: 'manual', x: 0, y: 0, z: 200, speed: 200, accel: 100, zone: 'z50', viaX: 100, viaY: 100, viaZ: 200 };
    case 'Honing':
      return { ...base, targetMode: 'manual', boreDiameter: 50, strokeLength: 100, spindleRpm: 300, reciprocationSpeed: 15, x: 0, y: 0, z: 200 };
    case 'MoveAbsJ':
      return { ...base, name: 'MoveAbsJ', joints: [0, 0, 0, 0, 0, 0], speed: 500, accel: 100, zone: 'z50' };
    case 'SearchL':
      return { ...base, name: 'SearchL', targetMode: 'manual', x: 0, y: 0, z: 100, speed: 10, accel: 50, zone: 'fine', stopCondition: 'DI_PartSensor', maxDistance: 200, storePose: true };
    case 'Wait':
      return { ...base, name: 'Wait 0.5s', duration: 0.5 };
    case 'SetDO':
      return { ...base, name: 'SetDO Gripper', signal: 'DO_Gripper', value: false, channel: 0 };
    case 'PulseDO':
      return { ...base, name: 'PulseDO', signal: 'DO_Light', channel: 2, pulseWidth: 200, pulseCount: 1 };
    case 'GetDI':
      return { ...base, name: 'GetDI Sensor', signal: 'DI_PartSensor', channel: 0 };
    case 'WaitDI':
      return { ...base, name: 'WaitDI', signal: 'DI_PartSensor', channel: 0, value: true, timeout: 5000 };
    case 'SetAO':
      return { ...base, name: 'SetAO Speed', signal: 'AO_Speed', channel: 0, analogValue: 5.0 };
    case 'GetAI':
      return { ...base, name: 'GetAI Force', signal: 'AI_ForceSensor', channel: 0 };
    case 'If':
      return { ...base, name: 'If', ifCondition: 'TRUE', conditionType: 'Expression', children: [], elseChildren: [] };
    case 'While':
      return { ...base, name: 'While TRUE', whileCondition: { type: 'TRUE' }, maxIterations: 0, breakOnError: true, children: [] };
    case 'For':
      return { ...base, name: 'For i = 1..10', loopVar: 'i', loopFrom: 1, loopTo: 10, loopStep: 1, children: [] };
    case 'Break':
      return { ...base, name: 'Break' };
    case 'Return':
      return { ...base, name: 'Return' };
    case 'Goto':
      return { ...base, name: 'Goto label', labelName: 'L1' };
    case 'Label':
      return { ...base, name: 'Label L1', labelName: 'L1' };
    case 'GripperOpen':
      return { ...base, name: 'Gripper Open', gripWidth: 80, gripForce: 20, gripSpeed: 100, toolRef: 'tool0' };
    case 'GripperClose':
      return { ...base, name: 'Gripper Close', gripWidth: 40, gripForce: 50, gripSpeed: 100, toolRef: 'tool0' };
    case 'WaitGrip':
      return { ...base, name: 'Wait Grip', gripForce: 30, timeout: 2000 };
    case 'ToolChange':
      return { ...base, name: 'Tool Change', toolRef: 'tool1' };
    case 'ResetError':
      return { ...base, name: 'Reset Error' };
    case 'StopProgram':
      return { ...base, name: 'Stop Program', reason: 'Manual stop' };
    case 'routine':
      return { ...base, name: 'NewRoutine', children: [], expanded: true };
    case 'CallProc':
      return { ...base, name: 'Call Routine', procName: 'Initialize' };
    case 'Module':
      return { ...base, name: 'Module', moduleName: 'MainModule', children: [], expanded: true };
    case 'SetVar':
      return { ...base, name: 'Set var', varName: 'counter', varType: 'num', varValue: '0' };
    case 'Increment':
      return { ...base, name: 'Increment counter', varName: 'counter', incrementAmount: 1 };
    case 'LoadData':
      return { ...base, name: 'Load Data' };
    default:
      return base;
  }
}

const categories = ['Motion', 'I/O', 'Flow', 'Robot', 'Structure', 'Variable'];

export const BlockLibraryPanel = () => {
  const { addConsoleEntry, programBlocks, setProgramBlocks, setSelectedBlockId, setRightPanelTab } = useAppState();

  const addBlock = useCallback((type: BlockNodeType) => {
    const newBlock = createDefaultBlock(type);
    const updated = [...programBlocks];
    const main = updated[0];
    if (main?.children) {
      main.children = [...main.children, newBlock];
    } else {
      updated.push(newBlock);
    }
    setProgramBlocks(updated);
    setSelectedBlockId(newBlock.id);
    setRightPanelTab('properties');
    addConsoleEntry('info', `Added ${type} block — edit in Properties →`);
  }, [programBlocks, setProgramBlocks, setSelectedBlockId, setRightPanelTab, addConsoleEntry]);

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Block Library</div>
      <div className="p-2 space-y-3 overflow-auto flex-1">
        {categories.map(cat => {
          const catBlocks = BLOCK_DEFS.filter(d => d.category === cat);
          return (
            <div key={cat}>
              <div className="text-[9px] text-muted-foreground/60 uppercase tracking-widest font-bold px-1 mb-1">{cat}</div>
              <div className="space-y-1">
                {catBlocks.map(def => (
                  <div
                    key={def.type}
                    className="block-card group"
                    onClick={() => addBlock(def.type)}
                    draggable
                    onDragStart={e => {
                      e.dataTransfer.setData('block-type', def.type);
                      e.dataTransfer.effectAllowed = 'copy';
                    }}
                  >
                    <div
                      className="w-7 h-7 rounded-md flex items-center justify-center shrink-0 text-[10px] font-bold"
                      style={{ background: def.bg, color: def.fg }}
                    >
                      {def.badge}
                    </div>
                    <div className="flex flex-col flex-1 min-w-0">
                      <span className="font-semibold text-[11px] text-foreground">{def.name}</span>
                      <span className="text-[9px] text-muted-foreground">{def.desc}</span>
                    </div>
                    <GripVertical size={12} className="text-muted-foreground/20 group-hover:text-muted-foreground/50 shrink-0 transition-colors" />
                  </div>
                ))}
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};
