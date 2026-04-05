import React from 'react';
import { useAppState } from '@/store/AppState';
import { AlertTriangle, AlertCircle, Info, Trash2, Play, StepForward } from 'lucide-react';

export const DiagnosticsPanel = () => {
  const { diagnostics, addDiagnostic, clearDiagnostics, addConsoleEntry, breakpoints } = useAppState();

  const addSampleError = () => {
    addDiagnostic({ severity: 'error', code: 'E1001', message: 'Unreachable position: Joint 3 limit exceeded at WP_Pick', line: 24, nodeId: 'wp2' });
    addDiagnostic({ severity: 'warning', code: 'W2003', message: 'Speed exceeds recommended limit for payload (v500 > v350)', line: 23 });
    addDiagnostic({ severity: 'info', code: 'I3001', message: 'Simulation cycle time: 4.2s' });
    addConsoleEntry('error', 'Diagnostics: 1 error, 1 warning found');
  };

  const severityIcon = (s: string) => {
    switch (s) {
      case 'error': return <AlertCircle size={14} className="text-status-error shrink-0" />;
      case 'warning': return <AlertTriangle size={14} className="text-status-warning shrink-0" />;
      default: return <Info size={14} className="text-status-info shrink-0" />;
    }
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        Diagnostics & Debug
        <div className="flex items-center gap-1">
          <button className="text-primary hover:text-primary/80 transition-colors text-[10px]" onClick={addSampleError}>Run Check</button>
          <button className="text-muted-foreground hover:text-foreground transition-colors" onClick={clearDiagnostics}><Trash2 size={12} /></button>
        </div>
      </div>
      <div className="flex-1 overflow-auto">
        {/* Debug controls */}
        <div className="flex items-center gap-2 px-3 py-2 border-b border-border">
          <button className="flex items-center gap-1 px-2 py-1 text-[10px] bg-secondary hover:bg-accent rounded-sm transition-colors"
            onClick={() => addConsoleEntry('info', '⏭ Step: MoveJ WP_Approach (line 23)')}>
            <StepForward size={12} /> Step
          </button>
          <span className="text-[10px] text-muted-foreground">Breakpoints: {breakpoints.size}</span>
        </div>

        {diagnostics.length === 0 ? (
          <div className="p-3 text-xs text-muted-foreground">
            No diagnostics. Click "Run Check" to verify program.
          </div>
        ) : (
          <div className="space-y-0.5 p-1">
            {diagnostics.map(d => (
              <div key={d.id} className="flex items-start gap-2 px-2 py-1.5 rounded-sm hover:bg-accent cursor-pointer transition-colors">
                {severityIcon(d.severity)}
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2">
                    <span className="text-[10px] text-muted-foreground font-mono">{d.code}</span>
                    {d.line && <span className="text-[10px] text-muted-foreground">Line {d.line}</span>}
                  </div>
                  <p className="text-xs mt-0.5">{d.message}</p>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};
