import React, { useState, useEffect } from 'react';
import { useRobotStateStore, HealthCheckResult } from '@/store/RosConnection';
import { 
  CheckCircle2, 
  XCircle, 
  Loader2, 
  AlertTriangle, 
  ShieldCheck, 
  Cpu, 
  Network, 
  Zap,
  Power
} from 'lucide-react';
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogDescription,
  DialogFooter,
} from "@/components/ui/dialog";
import { Button } from "@/components/ui/button";
import { Progress } from "@/components/ui/progress";
import { cn } from "@/lib/utils";

interface HealthCheckModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess: () => void;
}

export const HealthCheckModal: React.FC<HealthCheckModalProps> = ({ 
  isOpen, 
  onClose, 
  onSuccess 
}) => {
  const [status, setStatus] = useState<'idle' | 'running' | 'failed' | 'passed'>('idle');
  const [progress, setProgress] = useState(0);
  const { healthResults, runHealthCheck } = useRobotStateStore();

  const handleStartCheck = async () => {
    setStatus('running');
    setProgress(0);
    
    // Simulate progress while running real check
    const interval = setInterval(() => {
      setProgress(p => Math.min(p + 5, 90));
    }, 100);

    const isOk = await runHealthCheck();
    
    clearInterval(interval);
    setProgress(100);
    setStatus(isOk ? 'passed' : 'failed');
  };

  const getIcon = (name: string, ok: boolean) => {
    if (status === 'running') return <Loader2 className="h-4 w-4 animate-spin text-muted-foreground" />;
    return ok 
      ? <CheckCircle2 className="h-4 w-4 text-status-success" /> 
      : <XCircle className="h-4 w-4 text-status-error" />;
  };

  return (
    <Dialog open={isOpen} onOpenChange={(open) => !open && status !== 'running' && onClose()}>
      <DialogContent className="sm:max-w-[500px] glass-card border-glass-border">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2">
            <ShieldCheck className="text-primary" />
            System Pre-Flight Check (v8.0)
          </DialogTitle>
          <DialogDescription>
            Validating 11 critical components before enabling Live Hardware mode.
          </DialogDescription>
        </DialogHeader>

        <div className="py-4 space-y-4">
          {status === 'idle' ? (
            <div className="flex flex-col items-center justify-center py-8 text-center space-y-4 bg-accent/5 rounded-2xl border border-dashed border-glass-border">
              <Power className="h-12 w-12 text-muted-foreground/30" />
              <div className="space-y-1">
                <p className="text-sm font-medium">Ready for validation</p>
                <p className="text-xs text-muted-foreground">This ensures safe operation and connection stability.</p>
              </div>
            </div>
          ) : (
            <div className="space-y-3">
              <div className="flex justify-between items-end mb-1">
                <span className="text-xs font-medium text-muted-foreground">
                  {status === 'running' ? 'Scanning systems...' : status === 'passed' ? 'Verification Success' : 'System Failure Detected'}
                </span>
                <span className="text-xs font-mono text-muted-foreground">{progress}%</span>
              </div>
              <Progress value={progress} className="h-1.5" />
              
              <div className="max-h-[240px] overflow-auto pr-2 space-y-1.5 custom-scrollbar">
                {healthResults.length > 0 ? (
                  healthResults.map((res, i) => (
                    <div 
                      key={res.name} 
                      className={cn(
                        "flex items-center justify-between p-2 rounded-lg text-[11px] transition-colors",
                        res.ok ? "bg-status-success/5" : "bg-status-error/5 border border-status-error/10"
                      )}
                    >
                      <div className="flex items-center gap-2">
                        {getIcon(res.name, res.ok)}
                        <span className="font-mono text-muted-foreground">{res.name.replace(/_/g, ' ')}</span>
                      </div>
                      <span className={cn(
                        "text-[10px] italic",
                        res.ok ? "text-muted-foreground" : "text-status-error font-medium"
                      )}>
                        {res.ok ? 'OK' : res.detail}
                      </span>
                    </div>
                  ))
                ) : (
                  status === 'running' && [1,2,3,4,5].map(i => (
                    <div key={i} className="flex items-center justify-between p-2 animate-pulse bg-accent/5 rounded-lg">
                      <div className="h-3 w-32 bg-accent/10 rounded" />
                      <div className="h-3 w-12 bg-accent/10 rounded" />
                    </div>
                  ))
                )}
              </div>
            </div>
          )}
        </div>

        <DialogFooter className="sm:justify-between items-center">
          <div className="flex items-center gap-2">
            {status === 'failed' && (
              <span className="text-[10px] text-status-error flex items-center gap-1 font-medium">
                <AlertTriangle className="h-3 w-3" /> Fix issues to proceed
              </span>
            )}
          </div>
          <div className="flex gap-2">
            <Button variant="ghost" size="sm" onClick={onClose} disabled={status === 'running'}>
              Cancel
            </Button>
            {status === 'idle' || status === 'failed' ? (
              <Button size="sm" onClick={handleStartCheck} className="gap-2">
                <Zap className="h-4 w-4" /> Run System Check
              </Button>
            ) : status === 'passed' ? (
              <Button size="sm" onClick={onSuccess} className="bg-status-success hover:bg-status-success/90 gap-2">
                <CheckCircle2 className="h-4 w-4" /> Proceed to Live
              </Button>
            ) : null}
          </div>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
};
