import React, { useEffect, useRef } from 'react';
import { useAppState } from '@/store/AppState';
import { Trash2, ChevronDown, ChevronUp } from 'lucide-react';

export const ConsolePanel = () => {
  const { consoleEntries, consoleVisible, setConsoleVisible } = useAppState();
  const scrollRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [consoleEntries]);

  const typeColor = (type: string) => {
    switch (type) {
      case 'error': return 'text-console-error';
      case 'warning': return 'text-console-warning';
      case 'success': return 'text-status-ok';
      default: return 'text-console-text';
    }
  };

  return (
    <div className="flex flex-col border-t border-border" style={{ background: 'hsl(var(--console-bg))', height: consoleVisible ? 140 : 24 }}>
      <div className="flex items-center justify-between px-3 h-6 cursor-pointer shrink-0 border-b border-border"
        onClick={() => setConsoleVisible(!consoleVisible)}
        style={{ background: 'hsl(var(--panel-header))' }}>
        <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Console Output</span>
        <div className="flex items-center gap-1">
          <span className="text-[10px] text-muted-foreground">{consoleEntries.length} messages</span>
          {consoleVisible ? <ChevronDown size={12} className="text-muted-foreground" /> : <ChevronUp size={12} className="text-muted-foreground" />}
        </div>
      </div>
      {consoleVisible && (
        <div ref={scrollRef} className="flex-1 overflow-auto">
          {consoleEntries.map(entry => (
            <div key={entry.id} className="console-line flex items-start gap-2">
              <span className="text-muted-foreground shrink-0">[{entry.time}]</span>
              <span className={typeColor(entry.type)}>{entry.message}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};
