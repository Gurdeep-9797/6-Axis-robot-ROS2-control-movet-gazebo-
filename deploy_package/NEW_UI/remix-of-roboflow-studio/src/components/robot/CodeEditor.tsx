import React, { useState, useMemo } from 'react';
import { useAppState, ProgramBlockNode } from '@/store/AppState';
import { Play, ChevronRight, ChevronDown, Loader2, MoreHorizontal } from 'lucide-react';
import { BlockContextMenu } from './BlockContextMenu';
import { BADGE_CONFIG } from './BlockLibraryPanel';

// Generate RAPID code from program blocks (deterministic AST → Script)
function generateRAPID(blocks: ProgramBlockNode[], indent: number = 0): string[] {
  const lines: string[] = [];
  const pad = '  '.repeat(indent);

  for (const b of blocks) {
    if (b.type === 'Module') {
      lines.push(`${pad}MODULE ${b.moduleName || b.name}`);
      if (b.children) lines.push(...generateRAPID(b.children, indent + 1));
      lines.push(`${pad}ENDMODULE`);
    } else if (b.type === 'routine') {
      lines.push(`${pad}PROC ${b.name.replace(/\s/g, '_')}()`);
      if (b.children) lines.push(...generateRAPID(b.children, indent + 1));
      lines.push(`${pad}ENDPROC`);
      lines.push('');
    } else if (b.type === 'MoveJ' || b.type === 'MoveL') {
      const zone = b.zone || 'z50';
      const speed = b.speed || 500;
      const target = b.waypointId ? b.waypointId : `[${b.x || 0},${b.y || 0},${b.z || 0}]`;
      lines.push(`${pad}${b.type} ${target}, v${speed}, ${zone}, tool0;`);
    } else if (b.type === 'MoveC') {
      lines.push(`${pad}MoveC [${b.viaX || 0},${b.viaY || 0},${b.viaZ || 0}],[${b.x || 0},${b.y || 0},${b.z || 0}], v${b.speed || 200}, ${b.zone || 'z50'}, tool0;`);
    } else if (b.type === 'Honing') {
      lines.push(`${pad}Honing D${b.boreDiameter || 50} L${b.strokeLength || 100} S${b.spindleRpm || 300} v${b.reciprocationSpeed || 15};`);
    } else if (b.type === 'MoveAbsJ') {
      const joints = b.joints?.map(j => ((j * 180) / Math.PI).toFixed(1)).join(',') || '0,0,0,0,0,0';
      lines.push(`${pad}MoveAbsJ [${joints}], v${b.speed || 500}, ${b.zone || 'z50'}, tool0;`);
    } else if (b.type === 'SearchL') {
      lines.push(`${pad}SearchL \\Stop, ${b.stopCondition || 'DI_Sensor'}, [${b.x || 0},${b.y || 0},${b.z || 0}], v${b.speed || 10};`);
    } else if (b.type === 'Wait') {
      lines.push(`${pad}WaitTime ${b.duration || 0.5};`);
    } else if (b.type === 'SetDO') {
      lines.push(`${pad}SetDO ${b.signal || 'DO_Gripper'}, ${b.value ? '1' : '0'};`);
    } else if (b.type === 'PulseDO') {
      lines.push(`${pad}PulseDO \\PLength:=${b.pulseWidth || 200}, ${b.signal || 'DO_Light'};`);
    } else if (b.type === 'GetDI') {
      lines.push(`${pad}! Read ${b.signal || 'DI_Sensor'};`);
    } else if (b.type === 'WaitDI') {
      lines.push(`${pad}WaitDI ${b.signal || 'DI_Sensor'}, ${b.value ? '1' : '0'}${b.timeout ? ` \\MaxTime:=${b.timeout / 1000}` : ''};`);
    } else if (b.type === 'SetAO') {
      lines.push(`${pad}SetAO ${b.signal || 'AO_0'}, ${b.analogValue || 0};`);
    } else if (b.type === 'GetAI') {
      lines.push(`${pad}! Read ${b.signal || 'AI_0'};`);
    } else if (b.type === 'GripperOpen') {
      lines.push(`${pad}! GripperOpen width=${b.gripWidth || 80} force=${b.gripForce || 20};`);
      lines.push(`${pad}SetDO DO_Gripper, 0;`);
    } else if (b.type === 'GripperClose') {
      lines.push(`${pad}! GripperClose width=${b.gripWidth || 40} force=${b.gripForce || 50};`);
      lines.push(`${pad}SetDO DO_Gripper, 1;`);
    } else if (b.type === 'While') {
      const cond = b.whileCondition;
      const condStr = !cond || cond.type === 'TRUE' ? 'TRUE' : cond.type === 'DI' ? `DI${cond.channel} = ${cond.state}` : cond.expr || 'TRUE';
      lines.push(`${pad}WHILE ${condStr} DO`);
      if (b.children) lines.push(...generateRAPID(b.children, indent + 1));
      lines.push(`${pad}ENDWHILE`);
    } else if (b.type === 'If') {
      lines.push(`${pad}IF ${b.ifCondition || 'TRUE'} THEN`);
      if (b.children) lines.push(...generateRAPID(b.children, indent + 1));
      if (b.elseChildren && b.elseChildren.length > 0) {
        lines.push(`${pad}ELSE`);
        lines.push(...generateRAPID(b.elseChildren, indent + 1));
      }
      lines.push(`${pad}ENDIF`);
    } else if (b.type === 'For') {
      lines.push(`${pad}FOR ${b.loopVar || 'i'} FROM ${b.loopFrom || 1} TO ${b.loopTo || 10} STEP ${b.loopStep || 1} DO`);
      if (b.children) lines.push(...generateRAPID(b.children, indent + 1));
      lines.push(`${pad}ENDFOR`);
    } else if (b.type === 'Break') {
      lines.push(`${pad}Break;`);
    } else if (b.type === 'Return') {
      lines.push(`${pad}Return;`);
    } else if (b.type === 'CallProc') {
      lines.push(`${pad}${b.procName || 'Routine'}();`);
    } else if (b.type === 'SetVar') {
      lines.push(`${pad}${b.varName || 'var'} := ${b.varValue || '0'};`);
    } else if (b.type === 'Increment') {
      lines.push(`${pad}${b.varName || 'counter'} := ${b.varName || 'counter'} + ${b.incrementAmount || 1};`);
    } else if (b.type === 'ToolChange') {
      lines.push(`${pad}! ToolChange → ${b.toolRef || 'tool1'};`);
    } else if (b.type === 'ResetError') {
      lines.push(`${pad}! ResetError;`);
    } else if (b.type === 'StopProgram') {
      lines.push(`${pad}Stop; ! ${b.reason || 'Manual stop'}`);
    } else if (b.type === 'Goto') {
      lines.push(`${pad}GOTO ${b.labelName || 'L1'};`);
    } else if (b.type === 'Label') {
      lines.push(`${pad}${b.labelName || 'L1'}:`);
    }
  }
  return lines;
}

function classifyLine(line: string): string {
  const trimmed = line.trim();
  if (trimmed.startsWith('//') || trimmed.startsWith('!')) return 'text-code-comment';
  if (/^(PROC|ENDPROC|WHILE|ENDWHILE|IF|ENDIF|ELSE|THEN|DO|MODULE|ENDMODULE|FOR|ENDFOR|GOTO|BREAK|RETURN)/i.test(trimmed)) return 'text-code-keyword';
  if (/^(MoveJ|MoveL|MoveC|MoveAbsJ|SearchL)/.test(trimmed)) return 'text-code-function';
  if (/^(SetDO|GetDI|WaitTime|WaitDI|SetAO|PulseDO|Stop)/.test(trimmed)) return 'text-code-string';
  if (/^(GripperOpen|GripperClose|ToolChange|ResetError)/.test(trimmed)) return 'text-code-number';
  return 'text-foreground';
}

// Summary chip text for block tree nodes
function getSummaryChip(block: ProgramBlockNode): string {
  switch (block.type) {
    case 'MoveJ': case 'MoveL':
      return `(${block.x ?? 0},${block.y ?? 0},${block.z ?? 0}) v${block.speed ?? 500}`;
    case 'MoveC':
      return `arc v${block.speed ?? 200}`;
    case 'MoveAbsJ':
      return `joints v${block.speed ?? 500}`;
    case 'SearchL':
      return `→${block.stopCondition || 'DI'} v${block.speed ?? 10}`;
    case 'Wait':
      return `${(block.duration ?? 0.5).toFixed(2)}s`;
    case 'SetDO':
      return `${block.signal ?? 'DO'}=${block.value ? '1' : '0'}`;
    case 'PulseDO':
      return `${block.signal ?? 'DO'} ${block.pulseWidth ?? 200}ms`;
    case 'GetDI': case 'WaitDI':
      return `${block.signal ?? 'DI'}`;
    case 'SetAO':
      return `${block.signal ?? 'AO'}=${block.analogValue ?? 0}V`;
    case 'GetAI':
      return `${block.signal ?? 'AI'}`;
    case 'If':
      return block.ifCondition || 'TRUE';
    case 'While':
      return !block.whileCondition || block.whileCondition.type === 'TRUE' ? '∞ loop' :
        block.whileCondition.type === 'DI' ? `DI${block.whileCondition.channel}=${block.whileCondition.state}` :
        block.whileCondition.expr || 'TRUE';
    case 'For':
      return `${block.loopVar || 'i'}=${block.loopFrom ?? 1}..${block.loopTo ?? 10}`;
    case 'GripperOpen':
      return `w=${block.gripWidth ?? 80}mm`;
    case 'GripperClose':
      return `w=${block.gripWidth ?? 40}mm f=${block.gripForce ?? 50}N`;
    case 'CallProc':
      return block.procName || '';
    case 'SetVar':
      return `${block.varName}=${block.varValue}`;
    case 'Increment':
      return `${block.varName}+=${block.incrementAmount ?? 1}`;
    case 'routine':
      return `${block.children?.length ?? 0} instr`;
    case 'Module':
      return block.moduleName || '';
    case 'StopProgram':
      return block.reason || '';
    case 'ToolChange':
      return block.toolRef || '';
    default:
      return '';
  }
}

export const CodeEditor = () => {
  const { breakpoints, toggleBreakpoint, editorMode, setEditorMode, programBlocks } = useAppState();
  const [activeLine, setActiveLine] = useState(1);

  const codeLines = useMemo(() => {
    const header = ['// RoboForge v6.0 — Auto-generated RAPID', ''];
    const body = generateRAPID(programBlocks);
    return [...header, ...body];
  }, [programBlocks]);

  return (
    <div className="flex flex-col h-full" style={{ background: 'hsl(var(--code-bg))' }}>
      <div className="flex items-center justify-between px-3 py-1.5" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.05)' }}>
        <span className="text-[10px] text-muted-foreground font-mono">PickPlace_Main.mod</span>
        <div className="mode-toggle">
          <button className={`mode-toggle-btn ${editorMode === 'script' ? 'active' : ''}`} onClick={() => setEditorMode('script')}>Script</button>
          <button className={`mode-toggle-btn ${editorMode === 'block' ? 'active' : ''}`} onClick={() => setEditorMode('block')}>Blocks</button>
        </div>
      </div>

      {editorMode === 'script' ? (
        <div className="flex-1 overflow-auto font-mono text-[11px] leading-5">
          {codeLines.map((line, i) => {
            const lineNum = i + 1;
            const hasBP = breakpoints.has(lineNum);
            const isActive = activeLine === lineNum;
            return (
              <div key={i} className={`group flex items-center hover:bg-accent/15 cursor-text ${isActive ? 'bg-accent/20' : ''}`}
                onClick={() => setActiveLine(lineNum)}>
                <button className="w-4 h-5 flex items-center justify-center shrink-0"
                  style={{ opacity: hasBP ? 1 : 0 }}
                  onClick={(e) => { e.stopPropagation(); toggleBreakpoint(lineNum); }}>
                  {hasBP ? <div className="w-2 h-2 rounded-full bg-destructive shadow-[0_0_4px_hsl(var(--destructive))]" /> :
                    <div className="w-2 h-2 rounded-full border border-muted-foreground/20 opacity-0 group-hover:opacity-100" />}
                </button>
                <span className="w-8 text-right pr-3 shrink-0 select-none" style={{ color: 'hsl(var(--code-line-number))' }}>{lineNum}</span>
                <span className={classifyLine(line)}>{line}</span>
              </div>
            );
          })}
        </div>
      ) : (
        <BlockTreeView />
      )}
    </div>
  );
};

// Hierarchical block tree view
const BlockTreeView = () => {
  const { programBlocks, selectedBlockId, setSelectedBlockId, toggleBlockNode, executeBlock, setRightPanelTab, executingBlockId, setEditingBlockId } = useAppState();
  const [contextMenu, setContextMenu] = useState<{ blockId: string; x: number; y: number } | null>(null);

  const executableTypes = new Set(['MoveJ', 'MoveL', 'MoveC', 'Honing', 'MoveAbsJ', 'SearchL', 'Wait', 'SetDO', 'GetDI', 'PulseDO', 'WaitDI', 'GripperOpen', 'GripperClose', 'SetVar', 'Increment', 'SetAO', 'ToolChange', 'StopProgram', 'ResetError']);

  const renderBlock = (block: ProgramBlockNode, depth: number) => {
    const hasChildren = block.children && block.children.length > 0;
    const isSelected = selectedBlockId === block.id;
    const isExpanded = block.expanded !== false;
    const canExecute = executableTypes.has(block.type);
    const isRunning = executingBlockId === block.id;
    const badge = BADGE_CONFIG[block.type] || { badge: '•', bg: '#444', fg: '#aaa' };
    const summary = getSummaryChip(block);

    return (
      <div key={block.id}>
        <div
          className={`group flex items-center gap-1.5 py-1 pr-2 cursor-pointer transition-all duration-150 ${
            isSelected ? 'bg-primary/10' : 'hover:bg-accent/20'
          } ${isRunning ? 'tree-node-executing' : ''} ${block.hasBreakpoint ? 'border-l-2 border-destructive/50' : ''}`}
          style={{ paddingLeft: depth * 16 + 8 }}
          onClick={() => {
            setSelectedBlockId(block.id);
            setRightPanelTab('properties');
            if (hasChildren) toggleBlockNode(block.id);
          }}
          onDoubleClick={() => setEditingBlockId(block.id)}
          onContextMenu={e => {
            e.preventDefault();
            setContextMenu({ blockId: block.id, x: e.clientX, y: e.clientY });
          }}
        >
          {isRunning && <div className="w-1.5 h-1.5 rounded-full bg-destructive animate-pulse-led shrink-0" />}

          {hasChildren ? (
            <span className="text-muted-foreground w-3 shrink-0">
              {isExpanded ? <ChevronDown size={10} /> : <ChevronRight size={10} />}
            </span>
          ) : <span className="w-3 shrink-0" />}

          <span
            className="text-[9px] font-bold w-5 h-5 rounded-md flex items-center justify-center shrink-0"
            style={{ background: badge.bg, color: badge.fg }}
          >
            {badge.badge}
          </span>

          <span className="text-[11px] font-medium truncate">{block.name}</span>

          {summary && (
            <span className="text-[9px] text-muted-foreground font-mono ml-auto mr-1 truncate max-w-[120px] opacity-60">
              {summary}
            </span>
          )}

          {isRunning && <Loader2 size={10} className="text-status-ok animate-spin shrink-0" />}

          {canExecute && !isRunning && (
            <button className="cell-run-btn opacity-0 group-hover:opacity-100 shrink-0 !w-5 !h-5"
              onClick={(e) => { e.stopPropagation(); executeBlock(block.id); }}
              title="Run this block">
              <Play size={8} fill="currentColor" />
            </button>
          )}
        </div>
        {hasChildren && isExpanded && block.children!.map(child => renderBlock(child, depth + 1))}
        {block.type === 'If' && block.elseChildren && block.elseChildren.length > 0 && isExpanded && (
          <>
            <div className="text-[9px] text-muted-foreground/50 pl-8 py-0.5 italic" style={{ paddingLeft: depth * 16 + 24 }}>— else —</div>
            {block.elseChildren.map(child => renderBlock(child, depth + 1))}
          </>
        )}
      </div>
    );
  };

  return (
    <div className="flex-1 overflow-auto py-1 relative">
      <div className="px-3 py-1.5 flex items-center justify-between" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.04)' }}>
        <span className="text-[10px] text-muted-foreground">▶ run · right-click for options · double-click to edit</span>
      </div>
      {programBlocks.map(b => renderBlock(b, 0))}

      {contextMenu && (
        <BlockContextMenu
          blockId={contextMenu.blockId}
          x={contextMenu.x}
          y={contextMenu.y}
          onClose={() => setContextMenu(null)}
        />
      )}
    </div>
  );
};
