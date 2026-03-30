import React from 'react';
import { useAppState } from '@/store/AppState';

export const RobotConfigPanel = () => {
  const { robotType, setRobotType, projectName, addConsoleEntry } = useAppState();

  const robots = ['IRB 6700-235/2.65', 'IRB 1200-7/0.7', 'IRB 4600-60/2.05', 'UR10e', 'KUKA KR 210'];

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Robot Configuration</div>
      <div className="flex-1 overflow-auto p-3 space-y-4">
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Robot Model</span>
          <select
            className="w-full mt-1 bg-secondary border border-border rounded-sm px-2 py-1.5 text-xs text-foreground outline-none focus:border-primary"
            value={robotType}
            onChange={e => { setRobotType(e.target.value); addConsoleEntry('info', `Robot changed to ${e.target.value}`); }}
          >
            {robots.map(r => <option key={r} value={r}>{r}</option>)}
          </select>
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Specifications</span>
          {[
            ['Payload', '235 kg'], ['Reach', '2650 mm'], ['Axes', '6'],
            ['Repeatability', '±0.05 mm'], ['Weight', '1280 kg'], ['Protection', 'IP67'],
          ].map(([k, v]) => (
            <div key={k} className="property-field">
              <span className="property-label">{k}</span>
              <span className="text-xs">{v}</span>
            </div>
          ))}
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Joint Limits (°)</span>
          {[['J1', '±170'], ['J2', '+85/-65'], ['J3', '+70/-180'], ['J4', '±300'], ['J5', '±130'], ['J6', '±360']].map(([j, l]) => (
            <div key={j} className="property-field">
              <span className="property-label">{j}</span>
              <span className="text-xs">{l}</span>
            </div>
          ))}
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Tool (TCP)</span>
          <div className="property-field"><span className="property-label">Tool</span><span className="text-xs">Schunk PGN-plus 80</span></div>
          <div className="property-field"><span className="property-label">TCP Offset</span><span className="text-xs">[0, 0, 150] mm</span></div>
          <div className="property-field"><span className="property-label">Mass</span><span className="text-xs">2.4 kg</span></div>
        </div>
      </div>
    </div>
  );
};
