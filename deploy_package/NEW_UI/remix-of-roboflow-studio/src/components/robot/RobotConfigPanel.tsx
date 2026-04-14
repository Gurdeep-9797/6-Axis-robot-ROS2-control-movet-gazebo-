import React, { useState } from 'react';
import { useAppState } from '@/store/AppState';
import { ChevronDown, ChevronRight, Cpu, Zap, Settings2, CircuitBoard } from 'lucide-react';
import { Button } from '@/components/ui/button';
import { Switch } from '@/components/ui/switch';
import { Label } from '@/components/ui/label';
import { Input } from '@/components/ui/input';
import { HardwareConfigPage } from './HardwareConfigPage';

export const RobotConfigPanel = () => {
  const { robotType, setRobotType, addConsoleEntry } = useAppState();
  const [isHwConfigOpen, setIsHwConfigOpen] = useState(false);
  const robots = ['IRB 6700-235/2.65', 'IRB 1200-7/0.7', 'IRB 4600-60/2.05', 'UR10e', 'KUKA KR 210'];

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Robot Configuration</div>
      <div className="flex-1 overflow-auto p-3 space-y-4">
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Robot Model</span>
          <select className="glass-input w-full mt-1.5" value={robotType}
            onChange={e => { setRobotType(e.target.value); addConsoleEntry('info', `Robot changed to ${e.target.value}`); }}>
            {robots.map(r => <option key={r} value={r}>{r}</option>)}
          </select>
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Specifications</span>
          <div className="glass-surface rounded-lg p-2.5 space-y-1.5">
            {[['Payload', '235 kg'], ['Reach', '2650 mm'], ['Axes', '6'], ['Repeatability', '±0.05 mm'], ['Weight', '1280 kg'], ['Protection', 'IP67']].map(([k, v]) => (
              <div key={k} className="flex items-center justify-between">
                <span className="text-[10px] text-muted-foreground">{k}</span>
                <span className="text-[10px] font-medium">{v}</span>
              </div>
            ))}
          </div>
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Joint Limits (°)</span>
          <div className="glass-surface rounded-lg p-2.5 space-y-1.5">
            {[['J1', '±170'], ['J2', '+85/-65'], ['J3', '+70/-180'], ['J4', '±300'], ['J5', '±130'], ['J6', '±360']].map(([j, l]) => (
              <div key={j} className="flex items-center justify-between">
                <span className="text-[10px] text-muted-foreground font-mono">{j}</span>
                <span className="text-[10px]">{l}</span>
              </div>
            ))}
          </div>
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Tool (TCP)</span>
          <div className="glass-surface rounded-lg p-2.5 space-y-1.5">
            <div className="flex items-center justify-between"><span className="text-[10px] text-muted-foreground">Tool</span><span className="text-[10px]">Schunk PGN-plus 80</span></div>
            <div className="flex items-center justify-between"><span className="text-[10px] text-muted-foreground">TCP Offset</span><span className="text-[10px]">[0, 0, 150] mm</span></div>
            <div className="flex items-center justify-between"><span className="text-[10px] text-muted-foreground">Mass</span><span className="text-[10px]">2.4 kg</span></div>
          </div>
        </div>

        {/* ── Motor Hardware Layer ── */}
        <div className="space-y-2 pt-4 border-t border-glass-border">
          <div className="flex items-center justify-between">
            <span className="text-[10px] font-bold text-primary uppercase tracking-widest">Hardware Drive Layer</span>
            <Settings2 size={12} className="text-muted-foreground" />
          </div>

          {[1, 2, 3, 4, 5, 6].map(i => (
            <JointMotorConfig key={i} jointIndex={i} />
          ))}
        </div>

        {/* Edit Hardware Button */}
        <div className="pt-3 border-t border-glass-border">
          <button onClick={() => setIsHwConfigOpen(true)}
            className="w-full flex items-center justify-center gap-2 py-2.5 rounded-xl text-[11px] font-medium transition-all border border-cyan-500/30 hover:border-cyan-500/50 hover:bg-cyan-500/10 text-cyan-400">
            <CircuitBoard size={14} />
            Edit Hardware Configuration
          </button>
        </div>

        <HardwareConfigPage isOpen={isHwConfigOpen} onClose={() => setIsHwConfigOpen(false)} />
      </div>
    </div>
  );
};

const JointMotorConfig = ({ jointIndex }: { jointIndex: number }) => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [motorType, setMotorType] = useState<'DC' | 'BLDC'>('DC');

  return (
    <div className="glass-surface border-glass-border/30 rounded-lg overflow-hidden transition-all duration-300">
      <button 
        onClick={() => setIsExpanded(!isExpanded)}
        className="w-full flex items-center justify-between p-2.5 hover:bg-white/5 transition-colors"
      >
        <div className="flex items-center gap-2">
          {motorType === 'DC' ? <Zap size={12} className="text-status-warning" /> : <Cpu size={12} className="text-primary" />}
          <span className="text-[10px] font-medium font-mono">Joint {jointIndex} Motor</span>
        </div>
        <div className="flex items-center gap-3">
          <span className="text-[9px] bg-accent/20 px-1.5 py-0.5 rounded text-muted-foreground">{motorType}</span>
          {isExpanded ? <ChevronDown size={12} /> : <ChevronRight size={12} />}
        </div>
      </button>

      {isExpanded && (
        <div className="p-3 bg-black/20 border-t border-glass-border/10 space-y-3 animate-in slide-in-from-top-1 duration-200">
          <div className="flex items-center justify-between">
            <span className="text-[10px] text-muted-foreground">Driver Type</span>
            <div className="flex items-center gap-2 bg-accent/20 p-0.5 rounded-lg">
              <button 
                className={cn("text-[9px] px-2 py-0.5 rounded-md transition-all", motorType === 'DC' ? "bg-background shadow-sm text-foreground" : "text-muted-foreground")}
                onClick={() => setMotorType('DC')}
              >DC</button>
              <button 
                className={cn("text-[9px] px-2 py-0.5 rounded-md transition-all", motorType === 'BLDC' ? "bg-background shadow-sm text-foreground" : "text-muted-foreground")}
                onClick={() => setMotorType('BLDC')}
              >BLDC</button>
            </div>
          </div>

          <div className="grid grid-cols-2 gap-2">
            <div className="space-y-1">
              <Label className="text-[9px] text-muted-foreground uppercase">Kp Gain</Label>
              <Input type="number" step="0.1" defaultValue={1.2} className="h-7 text-[10px] bg-background/50" />
            </div>
            <div className="space-y-1">
              <Label className="text-[9px] text-muted-foreground uppercase">Ki Gain</Label>
              <Input type="number" step="0.01" defaultValue={0.05} className="h-7 text-[10px] bg-background/50" />
            </div>
            <div className="space-y-1">
              <Label className="text-[9px] text-muted-foreground uppercase">{motorType === 'DC' ? 'Max Current (A)' : 'Flux Linkage'}</Label>
              <Input type="number" step="0.1" defaultValue={5.0} className="h-7 text-[10px] bg-background/50" />
            </div>
            <div className="space-y-1">
              <Label className="text-[9px] text-muted-foreground uppercase">Gear Ratio</Label>
              <Input type="number" defaultValue={100} className="h-7 text-[10px] bg-background/50" />
            </div>
          </div>

          <div className="flex items-center justify-between pt-1">
            <span className="text-[9px] text-muted-foreground">Invert Direction</span>
            <Switch className="scale-75 origin-right" />
          </div>
        </div>
      )}
    </div>
  );
};

const cn = (...classes: any[]) => classes.filter(Boolean).join(' ');
