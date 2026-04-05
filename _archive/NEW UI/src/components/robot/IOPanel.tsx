import React from 'react';
import { useAppState } from '@/store/AppState';

export const IOPanel = () => {
  const { ioPoints, toggleIO } = useAppState();

  const digitalOuts = ioPoints.filter(io => io.type === 'digital-out');
  const digitalIns = ioPoints.filter(io => io.type === 'digital-in');
  const analogIns = ioPoints.filter(io => io.type === 'analog-in');
  const analogOuts = ioPoints.filter(io => io.type === 'analog-out');

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">I/O & Sensors</div>
      <div className="flex-1 overflow-auto p-3 space-y-4">
        {/* Digital Outputs */}
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Digital Outputs</span>
          <div className="space-y-1.5 mt-1.5">
            {digitalOuts.map(io => (
              <div key={io.id} className="flex items-center justify-between py-1">
                <div className="flex items-center gap-2">
                  <div className={`status-led ${io.value ? 'on' : 'off'}`} />
                  <span className="text-xs">{io.name}</span>
                </div>
                <button
                  className={`w-10 h-5 rounded-full transition-all duration-200 relative ${io.value ? 'bg-status-ok' : 'bg-secondary'}`}
                  onClick={() => toggleIO(io.id)}
                >
                  <div className={`absolute top-0.5 w-4 h-4 rounded-full bg-foreground transition-transform duration-200 ${io.value ? 'translate-x-5' : 'translate-x-0.5'}`} />
                </button>
              </div>
            ))}
          </div>
        </div>

        {/* Digital Inputs */}
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Digital Inputs</span>
          <div className="space-y-1.5 mt-1.5">
            {digitalIns.map(io => (
              <div key={io.id} className="flex items-center gap-2 py-1">
                <div className={`status-led ${io.value ? 'on' : 'off'}`} />
                <span className="text-xs">{io.name}</span>
                <span className={`text-[10px] ml-auto font-mono ${io.value ? 'text-status-ok' : 'text-muted-foreground'}`}>
                  {io.value ? 'HIGH' : 'LOW'}
                </span>
              </div>
            ))}
          </div>
        </div>

        {/* Analog Inputs */}
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Analog Inputs</span>
          <div className="space-y-2 mt-1.5">
            {analogIns.map(io => (
              <div key={io.id} className="space-y-0.5">
                <div className="flex items-center justify-between">
                  <span className="text-xs">{io.name}</span>
                  <span className="text-xs font-mono text-status-info">{(io.value as number).toFixed(1)}</span>
                </div>
                <div className="w-full h-1.5 bg-secondary rounded-full overflow-hidden">
                  <div className="h-full bg-status-info rounded-full transition-all" style={{ width: `${Math.min(100, (io.value as number))}%` }} />
                </div>
              </div>
            ))}
          </div>
        </div>

        {/* Analog Outputs */}
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Analog Outputs</span>
          <div className="space-y-2 mt-1.5">
            {analogOuts.map(io => (
              <div key={io.id} className="space-y-0.5">
                <div className="flex items-center justify-between">
                  <span className="text-xs">{io.name}</span>
                  <span className="text-xs font-mono">{(io.value as number).toFixed(0)}%</span>
                </div>
                <input type="range" min={0} max={100} value={io.value as number} className="w-full h-1 accent-primary" readOnly />
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};
