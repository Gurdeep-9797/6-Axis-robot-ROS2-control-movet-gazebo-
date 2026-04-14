import React from 'react';
import { useAppState, Waypoint } from '@/store/AppState';
import { Plus, Trash2, GripVertical, Crosshair } from 'lucide-react';

const MOTION_COLORS: Record<string, string> = {
  MoveJ: 'text-status-info',
  MoveL: 'text-status-ok',
  MoveC: 'text-status-warning',
};

export const MotionPlanningPanel = () => {
  const { waypoints, setWaypoints, selectedNodeId, setSelectedNodeId, addConsoleEntry, pickingMode, setPickingMode } = useAppState();

  const addWaypoint = () => {
    const id = `wp-${Date.now()}`;
    const newWP: Waypoint = {
      id, name: `WP_${waypoints.length + 1}`, x: 0, y: 0, z: 200,
      speed: 500, acceleration: 100, type: 'MoveJ', zone: 'z50',
      orientMode: 'quaternion', qw: 1, qx: 0, qy: 0, qz: 0, rx: 0, ry: 0, rz: 0,
    };
    setWaypoints([...waypoints, newWP]);
    setSelectedNodeId(id);
    addConsoleEntry('info', `Added waypoint: ${newWP.name}`);
  };

  const deleteWaypoint = (id: string) => {
    setWaypoints(waypoints.filter(w => w.id !== id));
    addConsoleEntry('info', `Deleted waypoint`);
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        Motion Planning
        <div className="flex items-center gap-1">
          <button className="glass-btn-primary text-[10px] px-2 py-0.5" onClick={() => setPickingMode('waypoint')} title="Pick from viewport">
            <Crosshair size={10} className="inline mr-0.5" /> Pick
          </button>
          <button className="text-primary hover:text-primary/80 transition-colors" onClick={addWaypoint} title="Add">
            <Plus size={13} />
          </button>
        </div>
      </div>

      {pickingMode === 'waypoint' && (
        <div className="mx-2 mt-2 section-card text-center">
          <Crosshair size={16} className="text-primary mx-auto mb-1 animate-pulse" />
          <p className="text-[10px] text-primary font-medium">Click in 3D viewport</p>
          <button className="glass-btn text-[10px] mt-1.5" onClick={() => setPickingMode('none')}>Cancel</button>
        </div>
      )}

      <div className="flex-1 overflow-auto p-2 space-y-1">
        {waypoints.map((wp) => (
          <div key={wp.id}
            className={`flex items-center gap-2 px-2.5 py-2 rounded-xl cursor-pointer transition-all duration-200 ${
              selectedNodeId === wp.id ? 'section-card border-primary/20' : 'hover:bg-accent/20'
            }`}
            onClick={() => setSelectedNodeId(wp.id)}>
            <GripVertical size={11} className="text-muted-foreground/30 cursor-grab shrink-0" />
            <div className="flex-1 min-w-0">
              <div className="flex items-center gap-1.5">
                <span className={`text-[9px] font-mono font-bold ${MOTION_COLORS[wp.type] || ''}`}>{wp.type}</span>
                <span className="text-[11px] font-medium truncate">{wp.name}</span>
              </div>
              <div className="text-[9px] text-muted-foreground/60">
                [{wp.x}, {wp.y}, {wp.z}] v{wp.speed} {wp.zone}
              </div>
            </div>
            <button className="text-muted-foreground/30 hover:text-destructive transition-colors shrink-0"
              onClick={(e) => { e.stopPropagation(); deleteWaypoint(wp.id); }}>
              <Trash2 size={11} />
            </button>
          </div>
        ))}
      </div>
    </div>
  );
};
