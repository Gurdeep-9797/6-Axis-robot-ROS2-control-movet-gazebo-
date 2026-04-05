import React from 'react';
import { useAppState } from '@/store/AppState';

export const PropertiesPanel = () => {
  const { selectedNodeId, waypoints, setWaypoints, robotType } = useAppState();

  const selectedWaypoint = waypoints.find(w => w.id === selectedNodeId);

  if (!selectedWaypoint) {
    return (
      <div className="flex flex-col h-full">
        <div className="panel-header">Properties</div>
        <div className="flex-1 p-3">
          <div className="space-y-3">
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Robot</span>
              <p className="text-xs mt-1">{robotType}</p>
            </div>
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Selection</span>
              <p className="text-xs text-muted-foreground mt-1">{selectedNodeId || 'None'}</p>
            </div>
            <p className="text-[10px] text-muted-foreground mt-4">Select a waypoint to edit properties</p>
          </div>
        </div>
      </div>
    );
  }

  const updateWP = (field: string, value: any) => {
    setWaypoints(waypoints.map(w => w.id === selectedWaypoint.id ? { ...w, [field]: value } : w));
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Properties — {selectedWaypoint.name}</div>
      <div className="flex-1 overflow-auto p-3 space-y-3">
        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Motion Type</span>
          <div className="mode-toggle mt-1">
            <button className={`mode-toggle-btn ${selectedWaypoint.type === 'MoveJ' ? 'active' : ''}`} onClick={() => updateWP('type', 'MoveJ')}>MoveJ</button>
            <button className={`mode-toggle-btn ${selectedWaypoint.type === 'MoveL' ? 'active' : ''}`} onClick={() => updateWP('type', 'MoveL')}>MoveL</button>
          </div>
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Position (mm)</span>
          {(['x', 'y', 'z'] as const).map(axis => (
            <div key={axis} className="property-field">
              <span className="property-label uppercase">{axis}</span>
              <input
                type="number"
                className="property-value w-20"
                value={selectedWaypoint[axis]}
                onChange={e => updateWP(axis, Number(e.target.value))}
              />
            </div>
          ))}
        </div>

        <div className="space-y-1.5">
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Parameters</span>
          <div className="property-field">
            <span className="property-label">Speed (mm/s)</span>
            <input type="number" className="property-value w-20" value={selectedWaypoint.speed} onChange={e => updateWP('speed', Number(e.target.value))} />
          </div>
          <div className="property-field">
            <span className="property-label">Accel (mm/s²)</span>
            <input type="number" className="property-value w-20" value={selectedWaypoint.acceleration} onChange={e => updateWP('acceleration', Number(e.target.value))} />
          </div>
        </div>

        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Joint Values (°)</span>
          <div className="space-y-1 mt-1">
            {['J1: 45.2', 'J2: -30.8', 'J3: 15.1', 'J4: 0.0', 'J5: -60.3', 'J6: 90.0'].map((j, i) => (
              <div key={i} className="property-field">
                <span className="property-label">{j.split(':')[0]}</span>
                <span className="text-xs text-foreground">{j.split(':')[1]}°</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};
