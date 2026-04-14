import React from 'react';
import { useAppState } from '@/store/AppState';

export const SettingsPanel = () => {
  const { settingsTab, setSettingsTab, ikMode, setIkMode, showBlending, setShowBlending, showTCPTrail, setShowTCPTrail, showWaypoints, setShowWaypoints, gizmoMode, setGizmoMode } = useAppState();

  const tabs = ['graphics', 'motion', 'visual', 'limits', 'controls', 'physics', 'integration'];

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Settings</div>
      <div className="flex overflow-x-auto" style={{ borderBottom: '1px solid hsl(var(--glass-border))' }}>
        {tabs.map(t => (
          <button key={t} className={`px-2.5 py-2 text-[10px] capitalize transition-all whitespace-nowrap ${
            settingsTab === t ? 'text-primary border-b-2 border-primary' : 'text-muted-foreground hover:text-foreground'
          }`} onClick={() => setSettingsTab(t)}>
            {t}
          </button>
        ))}
      </div>
      <div className="flex-1 overflow-auto p-3 space-y-3">
        {settingsTab === 'graphics' && (
          <>
            <GlassField label="Render API" value="DirectX 12" />
            <GlassField label="Target FPS" value="60" editable />
            <GlassSelect label="Quality" options={['Low', 'Medium', 'High', 'Ultra']} defaultVal="High" />
            <GlassToggle label="VSync" defaultVal={true} />
            <GlassToggle label="Anti-Aliasing" defaultVal={true} />
            <GlassToggle label="Shadows" defaultVal={true} />
          </>
        )}
        {settingsTab === 'motion' && (
          <>
            {/* ── IK Solver Mode ─────────────────────────────────────────── */}
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">IK Solver</span>
              <div className="glass-surface rounded-lg p-2 mt-1.5 space-y-1">
                {/* MoveIt — default */}
                <button
                  className={`w-full text-left px-2.5 py-2 rounded text-[11px] flex items-center gap-2 transition-all ${ikMode === 'moveit' ? 'bg-primary/20 text-primary border border-primary/40' : 'hover:bg-white/5 text-muted-foreground'}`}
                  onClick={() => setIkMode('moveit')}
                >
                  <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${ikMode === 'moveit' ? 'bg-primary' : 'bg-muted-foreground/40'}`} />
                  <span className="flex-1">MoveIt 2 <span className="text-[9px] opacity-60">(via /compute_ik — DEFAULT)</span></span>
                  {ikMode === 'moveit' && <span className="text-[8px] bg-primary/20 text-primary px-1.5 py-0.5 rounded">ACTIVE</span>}
                </button>
                {/* Divider with warning */}
                <div className="border-t border-white/10 pt-1">
                  <span className="text-[8px] text-yellow-400/70 px-1">⚠ Offline modes — bypass MoveIt, no backend required</span>
                </div>
                {/* Offline analytical */}
                <button
                  className={`w-full text-left px-2.5 py-2 rounded text-[11px] flex items-center gap-2 transition-all ${ikMode === 'offline-analytical' ? 'bg-yellow-500/10 text-yellow-300 border border-yellow-500/30' : 'hover:bg-white/5 text-muted-foreground'}`}
                  onClick={() => setIkMode('offline-analytical')}
                >
                  <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${ikMode === 'offline-analytical' ? 'bg-yellow-400' : 'bg-muted-foreground/40'}`} />
                  <span className="flex-1">Offline — Analytical <span className="text-[9px] opacity-60">(JS, fast, less accurate)</span></span>
                  {ikMode === 'offline-analytical' && <span className="text-[8px] bg-yellow-500/20 text-yellow-300 px-1.5 py-0.5 rounded">ACTIVE</span>}
                </button>
                {/* Offline numerical */}
                <button
                  className={`w-full text-left px-2.5 py-2 rounded text-[11px] flex items-center gap-2 transition-all ${ikMode === 'offline-numerical' ? 'bg-yellow-500/10 text-yellow-300 border border-yellow-500/30' : 'hover:bg-white/5 text-muted-foreground'}`}
                  onClick={() => setIkMode('offline-numerical')}
                >
                  <span className={`w-1.5 h-1.5 rounded-full flex-shrink-0 ${ikMode === 'offline-numerical' ? 'bg-yellow-400' : 'bg-muted-foreground/40'}`} />
                  <span className="flex-1">Offline — Numerical DLS <span className="text-[9px] opacity-60">(JS, slower, more accurate)</span></span>
                  {ikMode === 'offline-numerical' && <span className="text-[8px] bg-yellow-500/20 text-yellow-300 px-1.5 py-0.5 rounded">ACTIVE</span>}
                </button>
              </div>
            </div>
            <GlassField label="Default Speed" value="500 mm/s" editable />
            <GlassField label="Default Accel" value="100 mm/s²" editable />
            <GlassSelect label="Default Zone" options={['fine', 'z1', 'z5', 'z10', 'z50', 'z100']} defaultVal="z50" />
            <GlassToggle label="Enforce Joint Limits" defaultVal={true} />
            <GlassToggle label="Collision Detection" defaultVal={true} />
          </>
        )}
        {settingsTab === 'visual' && (
          <>
            <GlassToggle label="Show Blended Path" defaultVal={showBlending} onChange={setShowBlending} />
            <GlassToggle label="Show TCP Trail" defaultVal={showTCPTrail} onChange={setShowTCPTrail} />
            <GlassToggle label="Show Waypoint Markers" defaultVal={showWaypoints} onChange={setShowWaypoints} />
            <div>
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Gizmo Mode</span>
              <div className="mode-toggle mt-1.5">
                <button className={`mode-toggle-btn ${gizmoMode === 'off' ? 'active' : ''}`} onClick={() => setGizmoMode('off')}>Off</button>
                <button className={`mode-toggle-btn ${gizmoMode === 'translate' ? 'active' : ''}`} onClick={() => setGizmoMode('translate')}>Move</button>
                <button className={`mode-toggle-btn ${gizmoMode === 'rotate' ? 'active' : ''}`} onClick={() => setGizmoMode('rotate')}>Rotate</button>
              </div>
            </div>
            <div className="glass-surface rounded-lg p-2.5 mt-2">
              <span className="text-[10px] text-muted-foreground uppercase tracking-wider">Path Color Legend</span>
              <div className="space-y-1.5 mt-2">
                <div className="flex items-center gap-2"><div className="w-4 h-1 rounded-full bg-status-info" /><span className="text-[10px]">MoveJ — Joint interpolation</span></div>
                <div className="flex items-center gap-2"><div className="w-4 h-1 rounded-full" style={{ background: 'hsl(142 60% 45%)' }} /><span className="text-[10px]">MoveL — Linear</span></div>
                <div className="flex items-center gap-2"><div className="w-4 h-1 rounded-full" style={{ background: 'hsl(30 100% 50%)' }} /><span className="text-[10px]">MoveC — Circular</span></div>
              </div>
            </div>
          </>
        )}
        {settingsTab === 'limits' && (
          <>
            <GlassField label="Max TCP Speed" value="2000 mm/s" editable />
            <GlassField label="Max Joint Speed" value="250 °/s" editable />
            <GlassField label="Workspace Radius" value="2650 mm" />
            <GlassToggle label="Enforce Soft Limits" defaultVal={true} />
          </>
        )}
        {settingsTab === 'controls' && (
          <>
            <GlassSelect label="Mouse Mode" options={['Orbit', 'Pan', 'Fly']} defaultVal="Orbit" />
            <GlassField label="Pan Speed" value="1.0" editable />
            <GlassField label="Zoom Speed" value="1.0" editable />
            <GlassToggle label="Invert Y-Axis" defaultVal={false} />
          </>
        )}
        {settingsTab === 'physics' && (
          <>
            <GlassField label="Gravity" value="9.81 m/s²" editable />
            <GlassField label="Friction" value="0.5" editable />
            <GlassField label="Time Step" value="0.001 s" editable />
            <GlassSelect label="Solver" options={['Bullet', 'PhysX', 'ODE']} defaultVal="Bullet" />
          </>
        )}
        {settingsTab === 'integration' && (
          <>
            <GlassField label="SolidWorks API" value="Connected ✓" />
            <GlassField label="ROS 2 Bridge" value="Active" />
            <GlassField label="Controller IP" value="192.168.1.100" editable />
            <GlassField label="Controller Port" value="5000" editable />
            <GlassToggle label="Auto-sync CAD" defaultVal={true} />
          </>
        )}
      </div>
    </div>
  );
};

const GlassField = ({ label, value, editable }: { label: string; value: string; editable?: boolean }) => (
  <div className="flex items-center justify-between py-1.5">
    <span className="text-xs text-muted-foreground">{label}</span>
    {editable ? <input className="glass-input w-24 text-right" defaultValue={value} /> : <span className="text-xs">{value}</span>}
  </div>
);

const GlassSelect = ({ label, options, defaultVal }: { label: string; options: string[]; defaultVal: string }) => (
  <div className="flex items-center justify-between py-1.5">
    <span className="text-xs text-muted-foreground">{label}</span>
    <select className="glass-input w-24" defaultValue={defaultVal}>
      {options.map(o => <option key={o}>{o}</option>)}
    </select>
  </div>
);

const GlassToggle = ({ label, defaultVal, onChange }: { label: string; defaultVal: boolean; onChange?: (v: boolean) => void }) => {
  const [on, setOn] = React.useState(defaultVal);
  return (
    <div className="flex items-center justify-between py-1.5">
      <span className="text-xs text-muted-foreground">{label}</span>
      <button className={`glass-toggle ${on ? 'on' : ''}`} onClick={() => { setOn(!on); onChange?.(!on); }}>
        <div className={`glass-toggle-thumb ${on ? 'translate-x-4' : 'translate-x-0.5'}`} />
      </button>
    </div>
  );
};
