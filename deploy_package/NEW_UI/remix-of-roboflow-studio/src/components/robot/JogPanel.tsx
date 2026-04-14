import React, { useCallback } from 'react';
import { useAppState } from '@/store/AppState';
import { jointsToDegrees, forwardKinematics, solveIK } from '@/engine/IKSolver';
import { IRB6700_LIMITS } from '@/engine/MotionTypes';
import { Minus, Plus } from 'lucide-react';

const STEP_OPTIONS = [
  { label: '0.5°', value: 0.00873 },
  { label: '1°', value: 0.01745 },
  { label: '5°', value: 0.0873 },
  { label: '10°', value: 0.1745 },
];

const AXIS_COLORS: Record<string, string> = {
  X: 'hsl(0 80% 55%)',
  Y: 'hsl(142 65% 50%)',
  Z: 'hsl(210 100% 58%)',
};

export const JogPanel = () => {
  const { jointAngles, setJointAngles, jogMode, setJogMode, addConsoleEntry, jogStep, setJogStep } = useAppState();

  const jogJoint = useCallback((index: number, delta: number) => {
    const newAngles = [...jointAngles];
    const lim = IRB6700_LIMITS[index];
    const minRad = (lim.min * Math.PI) / 180;
    const maxRad = (lim.max * Math.PI) / 180;
    newAngles[index] = Math.max(minRad, Math.min(maxRad, newAngles[index] + delta));
    setJointAngles(newAngles);
  }, [jointAngles, setJointAngles]);

  const jogCartesian = useCallback((axis: string, delta: number) => {
    const fk = forwardKinematics(jointAngles);
    const target = { ...fk.position };
    const step = delta * 20; // mm
    if (axis === 'X') target.x += step;
    else if (axis === 'Y') target.y += step;
    else if (axis === 'Z') target.z += step;

    const result = solveIK(
      { position: target, orientation: { qw: 1, qx: 0, qy: 0, qz: 0 } },
      'numerical', jointAngles
    );
    if (result.success) {
      setJointAngles(result.joints);
    } else {
      addConsoleEntry('warning', `TCP jog ${axis}: IK unreachable`);
    }
  }, [jointAngles, setJointAngles, addConsoleEntry]);

  const degrees = jointsToDegrees(jointAngles);
  const fk = forwardKinematics(jointAngles);

  return (
    <div className="p-3 space-y-3">
      {/* Mode selector */}
      <div>
        <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold mb-1.5">Jog Mode</div>
        <div className="mode-toggle">
          <button className={`mode-toggle-btn ${jogMode === 'joint' ? 'active' : ''}`}
            onClick={() => setJogMode('joint')}>Joint</button>
          <button className={`mode-toggle-btn ${jogMode === 'cartesian' ? 'active' : ''}`}
            onClick={() => setJogMode('cartesian')}>Cartesian</button>
        </div>
      </div>

      {/* Step size */}
      <div>
        <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold mb-1.5">Step Size</div>
        <div className="flex gap-1">
          {STEP_OPTIONS.map(s => (
            <button key={s.label} className={`glass-btn flex-1 text-[10px] py-1 ${Math.abs(jogStep - s.value) < 0.001 ? 'active' : ''}`}
              onClick={() => setJogStep(s.value)}>{s.label}</button>
          ))}
        </div>
      </div>

      {jogMode === 'joint' ? (
        <div className="space-y-1.5">
          <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">Joint Control</div>
          {degrees.map((deg, i) => {
            const lim = IRB6700_LIMITS[i];
            const pct = ((deg - lim.min) / (lim.max - lim.min)) * 100;
            const nearLimit = pct < 5 || pct > 95;
            return (
              <div key={i} className="section-card p-2">
                <div className="flex items-center justify-between mb-1">
                  <span className="text-[10px] font-mono font-bold text-primary">J{i + 1}</span>
                  <span className={`text-[10px] font-mono tabular-nums ${nearLimit ? 'text-status-warning' : ''}`}>{deg.toFixed(1)}°</span>
                </div>
                <div className="flex items-center gap-1.5">
                  <button className="jog-btn w-7 h-7" onClick={() => jogJoint(i, -jogStep)}>
                    <Minus size={11} />
                  </button>
                  <div className="flex-1 relative h-1.5 rounded-full overflow-hidden" style={{ background: 'hsl(220 16% 13%)' }}>
                    <div className="absolute inset-y-0 left-0 rounded-full transition-all duration-100"
                      style={{
                        width: `${Math.max(1, Math.min(100, pct))}%`,
                        background: nearLimit
                          ? 'linear-gradient(90deg, hsl(var(--status-warning) / 0.6), hsl(var(--status-warning)))'
                          : 'linear-gradient(90deg, hsl(var(--primary) / 0.4), hsl(var(--primary)))',
                        boxShadow: `0 0 6px ${nearLimit ? 'hsl(var(--status-warning) / 0.3)' : 'hsl(var(--primary) / 0.3)'}`,
                      }} />
                  </div>
                  <button className="jog-btn w-7 h-7" onClick={() => jogJoint(i, jogStep)}>
                    <Plus size={11} />
                  </button>
                </div>
                <div className="flex items-center justify-between mt-0.5">
                  <span className="text-[8px] text-muted-foreground/50">{lim.min}°</span>
                  <span className="text-[8px] text-muted-foreground/50">{lim.max}°</span>
                </div>
              </div>
            );
          })}
        </div>
      ) : (
        <div className="space-y-3">
          <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">Linear Axes</div>
          {['X', 'Y', 'Z'].map(axis => (
            <div key={axis} className="section-card p-2">
              <div className="flex items-center justify-between mb-1">
                <span className="text-xs font-mono font-bold" style={{ color: AXIS_COLORS[axis] }}>{axis}</span>
                <span className="text-[10px] font-mono tabular-nums">
                  {axis === 'X' ? fk.position.x.toFixed(1) : axis === 'Y' ? fk.position.y.toFixed(1) : fk.position.z.toFixed(1)} mm
                </span>
              </div>
              <div className="flex items-center gap-2">
                <button className="jog-btn flex-1 h-7" onClick={() => jogCartesian(axis, -1)}>
                  <span className="text-[10px]">{axis}−</span>
                </button>
                <button className="jog-btn flex-1 h-7" onClick={() => jogCartesian(axis, 1)}>
                  <span className="text-[10px]">{axis}+</span>
                </button>
              </div>
            </div>
          ))}

          <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold">Rotation</div>
          <div className="grid grid-cols-3 gap-1.5">
            {['Rx', 'Ry', 'Rz'].map(axis => (
              <div key={axis} className="flex flex-col gap-1">
                <button className="jog-btn w-full h-7" onClick={() => jogCartesian(axis, 1)}>
                  <span className="text-[10px]">{axis}+</span>
                </button>
                <button className="jog-btn w-full h-7" onClick={() => jogCartesian(axis, -1)}>
                  <span className="text-[10px]">{axis}−</span>
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      {/* TCP readout */}
      <div className="section-card">
        <div className="text-[10px] text-muted-foreground uppercase tracking-widest font-semibold mb-1.5">TCP Position</div>
        <div className="grid grid-cols-3 gap-2">
          {[
            { label: 'X', val: fk.position.x, color: AXIS_COLORS.X },
            { label: 'Y', val: fk.position.y, color: AXIS_COLORS.Y },
            { label: 'Z', val: fk.position.z, color: AXIS_COLORS.Z },
          ].map(({ label, val, color }) => (
            <div key={label} className="text-center">
              <span className="text-[8px] font-semibold" style={{ color }}>{label}</span>
              <div className="text-[10px] font-mono tabular-nums">{val.toFixed(1)}</div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
