import React from 'react';
import { useAppState } from '@/store/AppState';
import { IKMode } from '@/engine/IKSolver';
import { Settings, X, Globe, Cpu, Radio, Activity, Target } from 'lucide-react';

const NavBtn = ({ active, onClick, icon: Icon, label }: { active: boolean; onClick: () => void; icon: any; label: string }) => (
  <button
    onClick={onClick}
    className={`flex items-center gap-3 px-4 py-3 rounded-xl font-medium text-sm w-full transition-all duration-150 ${
      active ? 'bg-primary/10 text-primary' : 'text-muted-foreground hover:bg-white/5'
    }`}
  >
    <Icon size={18} />
    {label}
  </button>
);

export const SettingsPage = ({ onClose }: { onClose: () => void }) => {
  const { globalStartPoint, setGlobalStartPoint, ikMode, setIkMode, jointAngles, addConsoleEntry } = useAppState();
  const [activeSection, setActiveSection] = React.useState<'environment' | 'controllers' | 'io' | 'logs'>('environment');

  const sections: { id: typeof activeSection; icon: any; label: string }[] = [
    { id: 'environment', icon: Globe, label: 'Environment' },
    { id: 'controllers', icon: Cpu, label: 'Controllers' },
    { id: 'io', icon: Radio, label: 'I/O Setup' },
    { id: 'logs', icon: Activity, label: 'Diagnostic Logs' },
  ];

  return (
    <div className="absolute inset-0 z-50 flex" style={{ background: 'hsl(var(--background))' }}>
      {/* Sidebar */}
      <div className="w-64 border-r border-glass-border p-4 flex flex-col gap-2 bg-black/20 flex-shrink-0">
        <h2 className="text-xl font-bold flex items-center gap-2 mb-6">
          <Settings size={24} className="text-primary" />
          Settings
        </h2>
        {sections.map(s => (
          <NavBtn key={s.id} active={activeSection === s.id} onClick={() => setActiveSection(s.id)} icon={s.icon} label={s.label} />
        ))}
      </div>

      {/* Main Content */}
      <div className="flex-1 flex flex-col relative overflow-hidden">
        <div className="p-4 border-b border-glass-border bg-black/10 flex justify-between items-center h-16">
          <h3 className="text-lg font-semibold capitalize">{activeSection} Settings</h3>
          <button onClick={onClose} className="p-2 hover:bg-white/10 rounded-full transition-colors">
            <X size={20} />
          </button>
        </div>

        <div className="flex-1 overflow-auto p-8 max-w-4xl mx-auto w-full space-y-8">

          {/* ── Environment ─────────────────────────────────── */}
          {activeSection === 'environment' && (
            <>
              <section className="space-y-4 section-card p-6">
                <h4 className="text-sm font-bold text-primary uppercase tracking-widest flex items-center gap-2 border-b border-glass-border pb-3">
                  <Target size={16} /> Global Start Point
                </h4>
                <p className="text-xs text-muted-foreground">Define the global 'Home' position in joint angles (°). Clicking "Home" in the toolbar moves the robot here.</p>
                <div className="grid grid-cols-6 gap-3 mt-4">
                  {globalStartPoint.map((angle, i) => (
                    <div key={i} className="flex flex-col gap-1.5">
                      <label className="text-[10px] text-muted-foreground font-mono text-center">J{i + 1}</label>
                      <input
                        type="number"
                        className="glass-input text-center py-2 text-sm"
                        value={angle}
                        onChange={e => {
                          const next = [...globalStartPoint];
                          next[i] = Number(e.target.value);
                          setGlobalStartPoint(next);
                        }}
                      />
                    </div>
                  ))}
                </div>
                <div className="flex justify-end mt-2">
                  <button
                    className="glass-btn text-[11px] px-4 py-2"
                    onClick={() => {
                      const degrees = jointAngles.map(r => parseFloat((r * (180 / Math.PI)).toFixed(1)));
                      setGlobalStartPoint(degrees);
                      addConsoleEntry('success', `✓ Global start set to: ${degrees.map(d => d.toFixed(1)).join(', ')}°`);
                    }}
                  >
                    <Target size={12} className="inline mr-1" />
                    Capture Current Robot Pose
                  </button>
                </div>
              </section>

              <section className="space-y-4 section-card p-6">
                <h4 className="text-sm font-bold text-primary uppercase tracking-widest flex items-center gap-2 border-b border-glass-border pb-3">
                  <Cpu size={16} /> Solver & Execution Engine
                </h4>
                <div>
                  <label className="text-xs block mb-2 text-muted-foreground">Inverse Kinematics Mode</label>
                  <select
                    className="glass-input w-full p-2"
                    value={ikMode}
                    onChange={e => setIkMode(e.target.value as IKMode)}
                  >
                    <option value="offline-numerical">Offline Iterative (Numerical) — No backend needed</option>
                    <option value="offline-analytical">Offline Exact (Analytical) — No backend needed</option>
                    <option value="moveit">Hardware / MoveIt Bridge — Requires Docker stack</option>
                  </select>
                  <p className="text-[10px] text-muted-foreground mt-1.5">
                    {ikMode === 'moveit'
                      ? '⚠ Requires Docker backend running (ws://localhost:9090)'
                      : '✓ No backend required — uses local JavaScript IK engine'}
                  </p>
                </div>
              </section>
            </>
          )}

          {/* ── Controllers ───────────────────────────────── */}
          {activeSection === 'controllers' && (
            <section className="space-y-4 section-card p-6">
              <h4 className="text-sm font-bold text-primary uppercase tracking-widest flex items-center gap-2 border-b border-glass-border pb-3">
                <Cpu size={16} /> Controller Configuration
              </h4>
              <div className="space-y-3">
                <div>
                  <label className="text-xs text-muted-foreground block mb-1">ROS Bridge WebSocket URL</label>
                  <input className="glass-input w-full" defaultValue="ws://localhost:9090" placeholder="ws://localhost:9090" />
                </div>
                <div>
                  <label className="text-xs text-muted-foreground block mb-1">Tunnel URL (for remote access)</label>
                  <input className="glass-input w-full" defaultValue="" placeholder="wss://your-tunnel-url.loca.lt" />
                </div>
                <div>
                  <label className="text-xs text-muted-foreground block mb-1">Joint velocity limit (°/s)</label>
                  <input type="number" className="glass-input w-32" defaultValue={120} />
                </div>
                <div>
                  <label className="text-xs text-muted-foreground block mb-1">Emergency stop delay (ms)</label>
                  <input type="number" className="glass-input w-32" defaultValue={50} />
                </div>
              </div>
            </section>
          )}

          {/* ── I/O Setup ─────────────────────────────────── */}
          {activeSection === 'io' && (
            <section className="space-y-4 section-card p-6">
              <h4 className="text-sm font-bold text-primary uppercase tracking-widest flex items-center gap-2 border-b border-glass-border pb-3">
                <Radio size={16} /> I/O Signal Mapping
              </h4>
              <p className="text-xs text-muted-foreground">Configure which ROS topics map to the physical I/O signals.</p>
              <div className="space-y-2 mt-3">
                {[
                  { id: 'do0', label: 'DO_Gripper', topic: '/io/DO_Gripper', type: 'digital-out' },
                  { id: 'do1', label: 'DO_Conveyor', topic: '/io/DO_Conveyor', type: 'digital-out' },
                  { id: 'di0', label: 'DI_PartSensor', topic: '/io/DI_PartSensor', type: 'digital-in' },
                  { id: 'di2', label: 'DI_Emergency', topic: '/io/DI_Emergency', type: 'digital-in' },
                  { id: 'ai0', label: 'AI_ForceSensor', topic: '/io/AI_ForceSensor', type: 'analog-in' },
                ].map(sig => (
                  <div key={sig.id} className="flex items-center gap-3 py-2 border-b border-white/5">
                    <span className="text-xs font-mono text-muted-foreground w-6">{sig.type === 'digital-out' ? 'DO' : sig.type === 'digital-in' ? 'DI' : 'AI'}</span>
                    <span className="text-xs font-medium w-36">{sig.label}</span>
                    <input className="glass-input flex-1 text-xs font-mono" defaultValue={sig.topic} />
                  </div>
                ))}
              </div>
            </section>
          )}

          {/* ── Logs ──────────────────────────────────────── */}
          {activeSection === 'logs' && (
            <section className="space-y-4 section-card p-6">
              <h4 className="text-sm font-bold text-primary uppercase tracking-widest flex items-center gap-2 border-b border-glass-border pb-3">
                <Activity size={16} /> Diagnostic Log Settings
              </h4>
              <div className="space-y-3">
                <div className="flex items-center justify-between">
                  <label className="text-xs text-muted-foreground">Max console log entries</label>
                  <input type="number" className="glass-input w-24 text-right" defaultValue={500} />
                </div>
                <div className="flex items-center justify-between">
                  <label className="text-xs text-muted-foreground">Show debug messages</label>
                  <input type="checkbox" className="accent-primary" defaultChecked />
                </div>
                <div className="flex items-center justify-between">
                  <label className="text-xs text-muted-foreground">Log to file</label>
                  <input type="checkbox" className="accent-primary" />
                </div>
              </div>
            </section>
          )}

        </div>
      </div>
    </div>
  );
};
