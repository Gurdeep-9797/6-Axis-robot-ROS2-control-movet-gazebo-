import React from 'react';
import { useAppState, TimelineEntry } from '@/store/AppState';
import { Trash2, Download } from 'lucide-react';

const TYPE_COLORS: Record<string, string> = {
  MoveJ: '#185FA5', MoveL: '#3B6D11', MoveC: '#085041', MoveAbsJ: '#0D3B6F',
  Wait: '#3C3489', SetDO: '#854F0B', PulseDO: '#9B6B0A', GetDI: '#993C1D',
  GripperOpen: '#1A6B8A', GripperClose: '#1A6B8A',
  SetVar: '#555', Increment: '#555',
  SearchL: '#8B4513', WaitDI: '#6B2FA0',
};

export const TimelinePanel = () => {
  const { timeline, clearTimeline } = useAppState();

  if (timeline.length === 0) {
    return (
      <div className="flex items-center justify-center h-full text-[11px] text-muted-foreground/50">
        Run program to see execution timeline
      </div>
    );
  }

  const minTime = timeline[0]?.startTime || 0;
  const maxTime = Math.max(...timeline.map(t => t.endTime || performance.now()));
  const totalDuration = maxTime - minTime || 1;

  return (
    <div className="flex flex-col h-full">
      <div className="flex items-center justify-between px-3 py-1 shrink-0" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.05)' }}>
        <span className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">Timeline</span>
        <div className="flex gap-1">
          <button className="glass-btn !px-1.5 !py-0.5 text-[9px]" onClick={clearTimeline} title="Clear">
            <Trash2 size={10} />
          </button>
          <button className="glass-btn !px-1.5 !py-0.5 text-[9px]" title="Export CSV">
            <Download size={10} />
          </button>
        </div>
      </div>
      <div className="flex-1 overflow-auto px-2 py-1">
        {/* Time axis */}
        <div className="flex items-center justify-between mb-1 text-[8px] text-muted-foreground/40 font-mono">
          <span>0ms</span>
          <span>{(totalDuration / 2).toFixed(0)}ms</span>
          <span>{totalDuration.toFixed(0)}ms</span>
        </div>
        {timeline.map((entry, i) => {
          const left = ((entry.startTime - minTime) / totalDuration) * 100;
          const width = (((entry.endTime || performance.now()) - entry.startTime) / totalDuration) * 100;
          const color = TYPE_COLORS[entry.blockType] || '#555';
          const duration = ((entry.endTime || performance.now()) - entry.startTime).toFixed(0);

          return (
            <div key={i} className="flex items-center gap-2 py-0.5 group">
              <span className="text-[9px] text-muted-foreground font-mono w-20 truncate shrink-0">{entry.blockName}</span>
              <div className="flex-1 relative h-4 rounded-sm overflow-hidden" style={{ background: 'hsl(220 16% 10%)' }}>
                <div
                  className="absolute inset-y-0 rounded-sm transition-all flex items-center px-1"
                  style={{
                    left: `${left}%`,
                    width: `${Math.max(2, width)}%`,
                    background: color,
                    opacity: entry.status === 'running' ? 0.8 : 0.6,
                  }}
                >
                  <span className="text-[7px] text-white font-mono truncate">{duration}ms</span>
                </div>
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
};
