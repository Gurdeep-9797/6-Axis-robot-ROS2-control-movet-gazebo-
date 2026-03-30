import React from 'react';
import { useAppState } from '@/store/AppState';
import { Plus, Trash2, GripVertical } from 'lucide-react';

export const MotionPlanningPanel = () => {
  const { waypoints, setWaypoints, selectedNodeId, setSelectedNodeId, addConsoleEntry } = useAppState();

  const addWaypoint = () => {
    const id = `wp-${Date.now()}`;
    const newWP = { id, name: `WP_New_${waypoints.length + 1}`, x: 0, y: 0, z: 200, speed: 500, acceleration: 100, type: 'MoveJ' as const };
    setWaypoints([...waypoints, newWP]);
    setSelectedNodeId(id);
    addConsoleEntry('info', `Added waypoint: ${newWP.name}`);
  };

  const deleteWaypoint = (id: string) => {
    setWaypoints(waypoints.filter(w => w.id !== id));
    addConsoleEntry('info', `Deleted waypoint: ${id}`);
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        Motion Planning
        <button className="text-primary hover:text-primary/80 transition-colors" onClick={addWaypoint} title="Add Waypoint">
          <Plus size={14} />
        </button>
      </div>
      <div className="flex-1 overflow-auto p-2 space-y-1">
        {waypoints.map((wp, i) => (
          <div
            key={wp.id}
            className={`flex items-center gap-2 px-2 py-2 rounded-sm cursor-pointer transition-colors ${selectedNodeId === wp.id ? 'bg-primary/15 text-primary' : 'hover:bg-accent'}`}
            onClick={() => setSelectedNodeId(wp.id)}
          >
            <GripVertical size={12} className="text-muted-foreground cursor-grab" />
            <div className="flex-1 min-w-0">
              <div className="text-xs font-medium truncate">{wp.name}</div>
              <div className="text-[10px] text-muted-foreground">
                {wp.type} — [{wp.x}, {wp.y}, {wp.z}] @ {wp.speed}mm/s
              </div>
            </div>
            <button className="text-muted-foreground hover:text-destructive transition-colors" onClick={(e) => { e.stopPropagation(); deleteWaypoint(wp.id); }}>
              <Trash2 size={12} />
            </button>
          </div>
        ))}
      </div>
    </div>
  );
};
