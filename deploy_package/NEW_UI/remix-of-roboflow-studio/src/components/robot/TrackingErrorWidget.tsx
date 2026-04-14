import React from 'react';
import { useRobotStateStore } from '@/store/RosConnection';
import { motion } from 'framer-motion';

interface ErrorBarProps {
  label: string;
  value: number;
  max: number;
  warnThreshold: number;
  critThreshold: number;
}

const ErrorBar: React.FC<ErrorBarProps> = ({ label, value, max, warnThreshold, critThreshold }) => {
  const absValue = Math.abs(value);
  const percentage = Math.min((absValue / max) * 100, 100);
  
  let color = 'bg-status-success';
  if (absValue > critThreshold) color = 'bg-status-error';
  else if (absValue > warnThreshold) color = 'bg-status-warning';

  return (
    <div className="space-y-1">
      <div className="flex justify-between text-[10px]">
        <span className="text-muted-foreground font-mono">{label}</span>
        <span className={absValue > warnThreshold ? 'text-foreground font-bold' : 'text-muted-foreground'}>
          {value.toFixed(2)}°
        </span>
      </div>
      <div className="h-1.5 w-full bg-accent/20 rounded-full overflow-hidden flex">
        <motion.div 
          initial={{ width: 0 }}
          animate={{ width: `${percentage}%` }}
          className={`h-full ${color} transition-colors duration-300`}
        />
      </div>
    </div>
  );
};

export const TrackingErrorWidget: React.FC = () => {
  const trackingError = useRobotStateStore(s => s.trackingError);
  
  if (!trackingError) return (
    <div className="p-3 text-[10px] text-muted-foreground italic text-center">
      Waiting for tracking data... (Live Mode only)
    </div>
  );

  return (
    <div className="space-y-3 p-3 glass-card rounded-xl">
      <div className="flex items-center justify-between">
        <span className="text-[11px] font-bold uppercase tracking-wider text-muted-foreground">Tracking Error</span>
        <div className="flex items-center gap-2">
          <span className="flex h-1.5 w-1.5 rounded-full bg-status-success animate-pulse" />
          <span className="text-[10px] text-muted-foreground">Live Sync</span>
        </div>
      </div>

      <div className="grid grid-cols-2 gap-x-4 gap-y-3">
        {trackingError.per_joint_error_deg.map((err, i) => (
          <ErrorBar 
            key={i} 
            label={`J${i+1}`} 
            value={err} 
            max={10} 
            warnThreshold={2} 
            critThreshold={5} 
          />
        ))}
      </div>

      <div className="pt-2 border-t border-glass-border flex justify-between items-center">
        <span className="text-[10px] text-muted-foreground">TCP Deviation</span>
        <span className={`text-xs font-mono font-bold ${trackingError.tcp_error_mm > 5 ? 'text-status-error' : 'text-status-success'}`}>
          {trackingError.tcp_error_mm.toFixed(1)} mm
        </span>
      </div>
      
      {trackingError.max_error_deg > 5 && (
        <div className="px-2 py-1 bg-status-error/10 border border-status-error/20 rounded text-[9px] text-status-error flex items-center gap-1.5">
          <span className="font-bold">CRITICAL:</span> High deviation on {trackingError.max_error_joint}
        </div>
      )}
    </div>
  );
};
