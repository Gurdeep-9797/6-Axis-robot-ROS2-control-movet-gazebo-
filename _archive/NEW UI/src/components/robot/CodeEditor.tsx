import React, { useState } from 'react';
import { useAppState } from '@/store/AppState';

const codeLines = [
  { text: '// PickAndPlace Program — IRB 6700', cls: 'text-code-comment' },
  { text: 'MODULE PickPlace_Main', cls: 'text-code-keyword' },
  { text: '', cls: '' },
  { text: '  CONST robtarget WP_Approach := [[450,200,300],[0,0,1,0]];', cls: '' },
  { text: '  CONST robtarget WP_Pick := [[450,200,50],[0,0,1,0]];', cls: '' },
  { text: '  CONST robtarget WP_Place := [[-300,400,50],[0,0,1,0]];', cls: '' },
  { text: '', cls: '' },
  { text: '  PROC main()', cls: 'text-code-function' },
  { text: '    Initialize;', cls: '' },
  { text: '    WHILE TRUE DO', cls: 'text-code-keyword' },
  { text: '      PickAndPlace;', cls: 'text-code-function' },
  { text: '      WaitTime 0.5;', cls: 'text-code-number' },
  { text: '    ENDWHILE', cls: 'text-code-keyword' },
  { text: '  ENDPROC', cls: 'text-code-keyword' },
  { text: '', cls: '' },
  { text: '  PROC Initialize()', cls: 'text-code-function' },
  { text: '    MoveAbsJ [[0,0,0,0,0,0]], v500, fine, tool0;', cls: '' },
  { text: '    SetDO DO_Gripper, 0;', cls: 'text-code-string' },
  { text: '    TPWrite "System initialized";', cls: 'text-code-string' },
  { text: '  ENDPROC', cls: 'text-code-keyword' },
  { text: '', cls: '' },
  { text: '  PROC PickAndPlace()', cls: 'text-code-function' },
  { text: '    MoveJ WP_Approach, v500, z50, tool0;', cls: '' },
  { text: '    MoveL WP_Pick, v200, fine, tool0;', cls: '' },
  { text: '    SetDO DO_Gripper, 1;', cls: 'text-code-string' },
  { text: '    WaitTime 0.2;', cls: 'text-code-number' },
  { text: '    MoveL WP_Approach, v500, z50, tool0;', cls: '' },
  { text: '    MoveJ WP_Place, v500, z50, tool0;', cls: '' },
  { text: '    SetDO DO_Gripper, 0;', cls: 'text-code-string' },
  { text: '    WaitTime 0.3;', cls: 'text-code-number' },
  { text: '  ENDPROC', cls: 'text-code-keyword' },
  { text: '', cls: '' },
  { text: 'ENDMODULE', cls: 'text-code-keyword' },
];

export const CodeEditor = () => {
  const { breakpoints, toggleBreakpoint, editorMode, setEditorMode } = useAppState();
  const [activeLine, setActiveLine] = useState(8);

  return (
    <div className="flex flex-col h-full" style={{ background: 'hsl(var(--code-bg))' }}>
      {/* Editor toolbar */}
      <div className="flex items-center justify-between px-2 py-1 border-b border-border">
        <div className="flex items-center gap-2">
          <span className="text-[10px] text-muted-foreground">PickPlace_Main.mod</span>
        </div>
        <div className="mode-toggle">
          <button className={`mode-toggle-btn ${editorMode === 'script' ? 'active' : ''}`} onClick={() => setEditorMode('script')}>Script</button>
          <button className={`mode-toggle-btn ${editorMode === 'block' ? 'active' : ''}`} onClick={() => setEditorMode('block')}>Blocks</button>
        </div>
      </div>

      {editorMode === 'script' ? (
        <div className="flex-1 overflow-auto font-mono text-xs leading-5">
          {codeLines.map((line, i) => {
            const lineNum = i + 1;
            const hasBP = breakpoints.has(lineNum);
            const isActive = activeLine === lineNum;
            return (
              <div
                key={i}
                className={`flex items-center hover:bg-accent/30 cursor-text ${isActive ? 'bg-accent/50' : ''}`}
                onClick={() => setActiveLine(lineNum)}
              >
                {/* Breakpoint gutter */}
                <button
                  className="w-4 h-5 flex items-center justify-center shrink-0 hover:opacity-100"
                  style={{ opacity: hasBP ? 1 : 0.2 }}
                  onClick={(e) => { e.stopPropagation(); toggleBreakpoint(lineNum); }}
                >
                  {hasBP && <div className="w-2 h-2 rounded-full bg-destructive" />}
                </button>
                {/* Line number */}
                <span className="w-8 text-right pr-3 shrink-0 select-none" style={{ color: 'hsl(var(--code-line-number))' }}>
                  {lineNum}
                </span>
                {/* Code */}
                <span className={line.cls || 'text-foreground'}>
                  {line.text}
                </span>
              </div>
            );
          })}
        </div>
      ) : (
        <BlockProgramView />
      )}
    </div>
  );
};

const BlockProgramView = () => {
  const { addConsoleEntry } = useAppState();
  const [expandedBlock, setExpandedBlock] = useState<string | null>(null);

  const programBlocks = [
    { id: 'init', name: 'Initialize', type: 'routine', color: 'border-l-status-ok' },
    { id: 'b-movej-home', name: 'MoveJ Home', type: 'move', color: 'border-l-block-move', parent: 'init' },
    { id: 'b-setdo-off', name: 'SetDO Gripper OFF', type: 'io', color: 'border-l-block-io', parent: 'init' },
    { id: 'loop', name: 'While TRUE', type: 'loop', color: 'border-l-block-loop' },
    { id: 'pick', name: 'PickAndPlace', type: 'routine', color: 'border-l-status-info', parent: 'loop' },
    { id: 'b-approach', name: 'MoveJ WP_Approach', type: 'move', color: 'border-l-block-move', parent: 'pick' },
    { id: 'b-pick', name: 'MoveL WP_Pick', type: 'move', color: 'border-l-block-move', parent: 'pick' },
    { id: 'b-grip', name: 'SetDO Gripper ON', type: 'io', color: 'border-l-block-io', parent: 'pick' },
    { id: 'b-place', name: 'MoveJ WP_Place', type: 'move', color: 'border-l-block-move', parent: 'pick' },
    { id: 'b-release', name: 'SetDO Gripper OFF', type: 'io', color: 'border-l-block-io', parent: 'pick' },
    { id: 'b-wait', name: 'Wait 0.5s', type: 'wait', color: 'border-l-block-wait', parent: 'loop' },
  ];

  const topLevel = programBlocks.filter(b => !b.parent);

  const renderBlock = (block: typeof programBlocks[0], depth: number) => {
    const children = programBlocks.filter(b => b.parent === block.id);
    const isExpanded = expandedBlock === block.id || children.length === 0;

    return (
      <div key={block.id} style={{ marginLeft: depth * 16 }}>
        <div
          className={`flex items-center gap-2 px-3 py-2 my-0.5 rounded-sm border-l-2 ${block.color} bg-secondary hover:bg-accent cursor-pointer transition-colors`}
          onClick={() => {
            setExpandedBlock(expandedBlock === block.id ? null : block.id);
            addConsoleEntry('info', `Selected block: ${block.name}`);
          }}
        >
          {children.length > 0 && (
            <span className="text-muted-foreground text-xs">{isExpanded ? '▼' : '▶'}</span>
          )}
          <span className="text-xs font-medium">{block.name}</span>
        </div>
        {isExpanded && children.map(child => renderBlock(child, depth + 1))}
      </div>
    );
  };

  return (
    <div className="flex-1 overflow-auto p-3">
      <p className="text-[10px] text-muted-foreground mb-2">Visual program flow — click to expand</p>
      {topLevel.map(b => renderBlock(b, 0))}
    </div>
  );
};
