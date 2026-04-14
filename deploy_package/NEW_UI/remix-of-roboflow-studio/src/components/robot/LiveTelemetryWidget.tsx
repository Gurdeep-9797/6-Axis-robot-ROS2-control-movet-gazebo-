import React, { useEffect, useState } from 'react';
import { Activity, Zap } from 'lucide-react';
import { backendConnector } from '@/services/BackendConnector';
import { useAppState } from '@/store/AppState';

export const LiveTelemetryWidget = () => {
  const [telemetry, setTelemetry] = useState<number[]>([]);
  const { mode } = useAppState();

  useEffect(() => {
    // We expect [duty_1, unused, unused, duty_2, unused, unused...] 
    // or [Id_1, Iq_1, unused...] 
    // For now we just graph the primary output value (duty cycle / command) per joint
    const updateCb = (data: number[]) => {
       setTelemetry(data);
    };
    backendConnector.onMotorTelemetry(updateCb);
    return () => {
       // Since onMotorTelemetry currently pushes to a single listener or doesn't have an offMotorTelemetry yet,
       // we should just let it be or implement offMotorTelemetry in BackendConnector if needed.
    };
  }, []);

  if (mode === 'offline') return null;

  return (
    <div className="glass-panel p-3 rounded-xl border border-glass-border">
      <div className="flex items-center gap-2 mb-3">
        <Activity size={14} className="text-primary" />
        <span className="text-xs font-semibold">Live PWM / Controllers</span>
      </div>
      <div className="grid grid-cols-2 gap-2">
        {[0, 1, 2, 3, 4, 5].map(j => {
          // The data array has 3 values per joint (Duty, x, x) or (Da, Db, Dc)
          const idx = j * 3;
          const val1 = telemetry.length > idx ? telemetry[idx] : 0;
          const isFwd = val1 >= 0;
          const absVal = Math.abs(val1) * 100;
          return (
            <div key={j} className="bg-black/20 rounded p-2 border border-white/5">
              <div className="flex justify-between items-center mb-1">
                 <span className="text-[10px] text-muted-foreground font-mono">J{j+1} CMD</span>
                 <span className={`text-[10px] font-mono ${isFwd ? 'text-green-400' : 'text-blue-400'}`}>
                    {isFwd ? '+' : '-'}{absVal.toFixed(1)}%
                 </span>
              </div>
              <div className="h-1.5 w-full bg-white/5 rounded-full overflow-hidden flex">
                 <div className="h-full bg-blue-500/80" style={{ width: !isFwd ? `${absVal}%` : '0%', float: 'right', marginLeft: 'auto' }} />
                 <div className="h-full bg-green-500/80" style={{ width: isFwd ? `${absVal}%` : '0%' }} />
              </div>
            </div>
          );
        })}
      </div>
      <div className="mt-3 flex items-center justify-between px-1">
         <span className="text-[9px] text-muted-foreground flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-green-500/80" /> FWD (Positive Duty)</span>
         <span className="text-[9px] text-muted-foreground flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-blue-500/80" /> REV (Negative Duty)</span>
      </div>
    </div>
  );
};
