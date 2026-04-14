import React, { useEffect, useRef, useState } from 'react';
import { useAppState } from '@/store/AppState';
import { ChevronDown, ChevronUp, Trash2 } from 'lucide-react';
import { TimelinePanel } from './TimelinePanel';

export const ConsolePanel = () => {
  const { consoleEntries, consoleVisible, setConsoleVisible } = useAppState();
  const scrollRef = useRef<HTMLDivElement>(null);
  const [activeTab, setActiveTab] = useState<'console' | 'timeline' | 'debug'>('console');

  useEffect(() => {
    if (scrollRef.current) scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
  }, [consoleEntries]);

  const typeColor = (type: string) => {
    switch (type) {
      case 'error': return 'text-console-error';
      case 'warning': return 'text-console-warning';
      case 'success': return 'text-status-ok';
      case 'debug': return 'text-muted-foreground';
      default: return 'text-console-text';
    }
  };

  const typePrefix = (type: string) => {
    switch (type) {
      case 'error': return 'ERR';
      case 'warning': return 'WRN';
      case 'success': return ' OK';
      case 'debug': return 'DBG';
      default: return 'INF';
    }
  };

  // When used as a LayoutSlot panel, always expand to fill the available space
  return (
    <div className="flex flex-col h-full" style={{ background: 'hsl(var(--console-bg))' }}>
      {/* Tab bar */}
      <div className="flex items-center justify-between px-1 h-7 shrink-0 border-b border-white/5"
        style={{ background: 'rgba(0,0,0,0.35)' }}>
        <div className="flex items-center gap-0">
          {(['console', 'timeline', 'debug'] as const).map(tab => (
            <button key={tab} className={`px-3 py-1 text-[10px] font-semibold uppercase tracking-widest transition-all ${
              activeTab === tab ? 'text-primary border-b border-primary' : 'text-muted-foreground/50 hover:text-muted-foreground'
            }`} onClick={() => setActiveTab(tab)}>
              {tab}
            </button>
          ))}
        </div>
        <div className="flex items-center gap-2 pr-2">
          <span className="text-[9px] text-muted-foreground/40 font-mono">{consoleEntries.length} entries</span>
          <button
            className="p-0.5 hover:bg-white/10 rounded text-muted-foreground/40 hover:text-foreground transition-colors"
            title="Clear console"
            onClick={() => {/* handled by AppState addConsoleEntry */}}>
            <Trash2 size={9} />
          </button>
        </div>
      </div>

      {/* Content area — always full height */}
      {activeTab === 'console' && (
        <div ref={scrollRef} className="flex-1 overflow-auto py-0.5 min-h-0">
          {consoleEntries.length === 0 ? (
            <div className="flex items-center justify-center h-full text-[10px] text-muted-foreground/30">
              No output yet
            </div>
          ) : (
            consoleEntries.map(entry => (
              <div key={entry.id} className="console-line flex items-start gap-2 hover:bg-white/3 px-2">
                <span className="text-muted-foreground/30 shrink-0 text-[9px] font-mono tabular-nums w-14">{entry.time}</span>
                <span className={`${typeColor(entry.type)} shrink-0 text-[9px] font-bold font-mono w-7`}>{typePrefix(entry.type)}</span>
                <span className={`${typeColor(entry.type)} text-[11px] break-all`}>{entry.message}</span>
              </div>
            ))
          )}
        </div>
      )}
      {activeTab === 'timeline' && (
        <div className="flex-1 overflow-hidden min-h-0">
          <TimelinePanel />
        </div>
      )}
      {activeTab === 'debug' && (
        <DebugPanel />
      )}
    </div>
  );
};

const DebugPanel = () => {
  const { variableWatch, executingBlockId, programBlocks } = useAppState();

  return (
    <div className="flex-1 overflow-auto p-2 space-y-2">
      <div>
        <div className="text-[9px] text-muted-foreground/60 uppercase tracking-widest mb-1">Variable Watch</div>
        <div className="space-y-0.5">
          {Object.entries(variableWatch).map(([key, val]) => (
            <div key={key} className="flex items-center justify-between px-2 py-0.5 rounded-md" style={{ background: 'hsl(220 18% 10%)' }}>
              <span className="text-[10px] font-mono text-muted-foreground">{key}</span>
              <span className="text-[10px] font-mono text-foreground tabular-nums">{String(val)}</span>
            </div>
          ))}
        </div>
      </div>
      <div>
        <div className="text-[9px] text-muted-foreground/60 uppercase tracking-widest mb-1">Execution State</div>
        <div className="px-2 py-1 rounded-md text-[10px]" style={{ background: 'hsl(220 18% 10%)' }}>
          {executingBlockId ? (
            <span className="text-status-ok">▶ Executing: {executingBlockId}</span>
          ) : (
            <span className="text-muted-foreground">● Idle</span>
          )}
        </div>
      </div>
    </div>
  );
};
