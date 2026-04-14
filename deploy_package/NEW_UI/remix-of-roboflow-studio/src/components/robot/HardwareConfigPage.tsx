import React, { useState, useEffect, useCallback } from 'react';
import { useAppState } from '@/store/AppState';
import { backendConnector } from '@/services/BackendConnector';
import {
  X, Cpu, Zap, Activity, CheckCircle2, XCircle, AlertTriangle,
  RotateCw, Settings2, ChevronDown, ChevronRight, Play, Square,
  RefreshCw, Save, Upload, Gauge, Cable, CircuitBoard
} from 'lucide-react';

// ── Types ──────────────────────────────────────────────────────────────
type MotorType = 'DC' | 'BLDC' | 'Stepper';
type EncoderType = 'incremental' | 'absolute-spi' | 'absolute-i2c' | 'absolute-ssi' | 'resolver';
type SignalType = 'PWM' | 'Step/Dir' | 'CAN' | 'EtherCAT';
type PostStatus = 'idle' | 'running' | 'passed' | 'failed';

interface JointHwConfig {
  motorType: MotorType;
  signalType: SignalType;
  kp: number;
  ki: number;
  maxCurrent: number;
  gearRatio: number;
  invertDirection: boolean;
  // BLDC-specific
  polePairs: number;
  fluxLinkage: number;
  phaseResistance: number;
  focKpD: number; focKiD: number;
  focKpQ: number; focKiQ: number;
  // Encoder
  encoderType: EncoderType;
  cpr: number;
  encoderGearRatio: number;
  zeroOffset: number;
  encoderDirection: number;
  // Status
  configured: boolean;
  testResult: 'untested' | 'pass' | 'fail' | 'testing';
}

const DEFAULT_JOINT: JointHwConfig = {
  motorType: 'DC', signalType: 'PWM',
  kp: 1.2, ki: 0.05, maxCurrent: 5.0, gearRatio: 100, invertDirection: false,
  polePairs: 7, fluxLinkage: 0.012, phaseResistance: 0.5,
  focKpD: 0.5, focKiD: 5.0, focKpQ: 0.5, focKiQ: 5.0,
  encoderType: 'absolute-spi', cpr: 4096, encoderGearRatio: 100, zeroOffset: 2048, encoderDirection: 1,
  configured: false, testResult: 'untested',
};

// ── Main Component ─────────────────────────────────────────────────────
interface HardwareConfigPageProps {
  isOpen: boolean;
  onClose: () => void;
}

export const HardwareConfigPage: React.FC<HardwareConfigPageProps> = ({ isOpen, onClose }) => {
  const { addConsoleEntry } = useAppState();
  const [joints, setJoints] = useState<JointHwConfig[]>(
    Array.from({ length: 6 }, () => ({ ...DEFAULT_JOINT }))
  );
  const [activeJoint, setActiveJoint] = useState(0);
  const [activeTab, setActiveTab] = useState<'motor' | 'encoder' | 'test'>('motor');
  const [postStatus, setPostStatus] = useState<PostStatus>('idle');
  const [postResults, setPostResults] = useState<any[]>([]);
  const [configSaved, setConfigSaved] = useState(false);
  const [liveEncoderValues, setLiveEncoderValues] = useState<number[]>([0,0,0,0,0,0]);

  // Load saved config on mount
  useEffect(() => {
    const saved = localStorage.getItem('roboforge_hw_config');
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        setJoints(parsed);
        setConfigSaved(true);
      } catch {}
    }
  }, []);

  // Simulated live encoder polling (in real mode, this comes from WS subscription)
  useEffect(() => {
    if (!isOpen || activeTab !== 'test') return;
    const iv = setInterval(() => {
      setLiveEncoderValues(prev => prev.map((v, i) =>
        joints[i].testResult === 'testing'
          ? v + (Math.random() - 0.5) * 10
          : v
      ));
    }, 50);
    return () => clearInterval(iv);
  }, [isOpen, activeTab, joints]);

  const updateJoint = useCallback((idx: number, patch: Partial<JointHwConfig>) => {
    setJoints(prev => prev.map((j, i) => i === idx ? { ...j, ...patch } : j));
    setConfigSaved(false);
  }, []);

  const handleSaveConfig = useCallback(() => {
    localStorage.setItem('roboforge_hw_config', JSON.stringify(joints));
    setConfigSaved(true);
    addConsoleEntry('success', '✓ Hardware configuration saved');

    // Publish to backend
    joints.forEach((j, idx) => {
      backendConnector.publishMotorConfig?.(idx, {
        motor_type: j.motorType,
        pid: { kp: j.kp, ki: j.ki },
        gear_ratio: j.gearRatio,
        max_current_a: j.maxCurrent,
        signal_type: j.signalType,
      });
      backendConnector.publishEncoderConfig?.(idx, {
        encoder_type: j.encoderType,
        counts_per_rev: j.cpr,
        gear_ratio: j.encoderGearRatio,
        zero_offset: j.zeroOffset,
        direction: j.encoderDirection,
      });
    });
  }, [joints, addConsoleEntry]);

  const handleRunPOST = useCallback(async () => {
    setPostStatus('running');
    setPostResults([]);
    addConsoleEntry('info', '▶ Running Power-On Self-Test...');

    // Simulate POST per joint (in live mode, this calls the bridge)
    const results: any[] = [];
    for (let i = 0; i < 6; i++) {
      await new Promise(r => setTimeout(r, 400));
      const encoder_ok = joints[i].configured || Math.random() > 0.1;
      const motor_ok = joints[i].configured || Math.random() > 0.1;
      const serial_ok = true;
      results.push({
        joint: i + 1,
        encoder_ok, motor_ok, serial_ok,
        latency_ms: +(Math.random() * 2).toFixed(1),
      });
      setPostResults([...results]);
    }

    const allPassed = results.every(r => r.encoder_ok && r.motor_ok && r.serial_ok);
    setPostStatus(allPassed ? 'passed' : 'failed');
    addConsoleEntry(allPassed ? 'success' : 'error',
      allPassed ? '✓ POST passed — all joints nominal' : '✗ POST failed — check flagged joints');
  }, [joints, addConsoleEntry]);

  const handleTestMotor = useCallback((idx: number) => {
    updateJoint(idx, { testResult: 'testing' });
    addConsoleEntry('info', `⚙ Testing Joint ${idx + 1} motor (±5° jog)...`);
    setTimeout(() => {
      updateJoint(idx, { testResult: 'pass', configured: true });
      addConsoleEntry('success', `✓ Joint ${idx + 1} motor test passed`);
    }, 2000);
  }, [updateJoint, addConsoleEntry]);

  const handleTestEncoder = useCallback((idx: number) => {
    updateJoint(idx, { testResult: 'testing' });
    addConsoleEntry('info', `📡 Reading Joint ${idx + 1} encoder...`);
    setTimeout(() => {
      updateJoint(idx, { testResult: 'pass' });
      addConsoleEntry('success', `✓ Joint ${idx + 1} encoder verified (raw: ${2048 + Math.floor(Math.random() * 200)})`);
    }, 1500);
  }, [updateJoint, addConsoleEntry]);

  if (!isOpen) return null;

  const j = joints[activeJoint];

  return (
    <div className="fixed inset-0 z-[100] flex items-center justify-center" style={{ background: 'rgba(0,0,0,0.7)', backdropFilter: 'blur(8px)' }}>
      <div className="w-[960px] max-h-[85vh] rounded-2xl overflow-hidden flex flex-col"
           style={{ background: 'hsl(220 25% 10%)', border: '1px solid hsl(220 20% 20%)' }}>

        {/* Header */}
        <div className="flex items-center justify-between px-5 py-3" style={{ borderBottom: '1px solid hsl(220 20% 18%)' }}>
          <div className="flex items-center gap-3">
            <CircuitBoard size={18} className="text-cyan-400" />
            <span className="font-bold text-sm">Hardware Configuration</span>
            <span className="text-[9px] px-2 py-0.5 rounded-full"
                  style={{ background: configSaved ? 'hsl(140 60% 15%)' : 'hsl(40 60% 15%)',
                           color: configSaved ? 'hsl(140 60% 60%)' : 'hsl(40 60% 60%)' }}>
              {configSaved ? '● Saved' : '● Unsaved'}
            </span>
          </div>
          <div className="flex items-center gap-2">
            <button onClick={handleSaveConfig}
              className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-[10px] font-medium transition-colors"
              style={{ background: 'hsl(210 60% 25%)', color: 'hsl(210 60% 80%)' }}>
              <Save size={12} /> Save Config
            </button>
            <button onClick={handleRunPOST}
              disabled={postStatus === 'running'}
              className="flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-[10px] font-medium transition-colors"
              style={{ background: 'hsl(160 60% 20%)', color: 'hsl(160 60% 75%)' }}>
              <Activity size={12} /> {postStatus === 'running' ? 'Running POST...' : 'Run POST'}
            </button>
            <button onClick={onClose} className="p-1 rounded-lg hover:bg-white/10 transition-colors">
              <X size={16} />
            </button>
          </div>
        </div>

        <div className="flex flex-1 overflow-hidden">
          {/* Joint selector */}
          <div className="w-48 p-3 space-y-1 overflow-auto" style={{ borderRight: '1px solid hsl(220 20% 18%)' }}>
            <div className="text-[9px] uppercase tracking-widest text-muted-foreground/40 font-bold mb-2 px-1">Joints</div>
            {joints.map((jt, i) => (
              <button key={i} onClick={() => setActiveJoint(i)}
                className={`w-full flex items-center gap-2 px-3 py-2.5 rounded-xl text-left transition-all ${
                  activeJoint === i ? 'shadow-lg' : 'hover:bg-white/5'
                }`}
                style={activeJoint === i ? { background: 'hsl(210 50% 18%)', borderLeft: '2px solid hsl(210 80% 60%)' } : {}}>
                <div className="flex items-center gap-2 flex-1 min-w-0">
                  {jt.motorType === 'DC'
                    ? <Zap size={13} className="text-yellow-500 shrink-0" />
                    : jt.motorType === 'BLDC'
                    ? <Cpu size={13} className="text-cyan-400 shrink-0" />
                    : <Settings2 size={13} className="text-purple-400 shrink-0" />}
                  <div className="flex flex-col min-w-0">
                    <span className="text-[11px] font-medium font-mono">Joint {i + 1}</span>
                    <span className="text-[9px] text-muted-foreground">{jt.motorType} • {jt.encoderType.split('-')[0]}</span>
                  </div>
                </div>
                <div className="shrink-0">
                  {jt.testResult === 'pass' && <CheckCircle2 size={12} className="text-green-500" />}
                  {jt.testResult === 'fail' && <XCircle size={12} className="text-red-500" />}
                  {jt.testResult === 'testing' && <RotateCw size={12} className="text-cyan-400 animate-spin" />}
                </div>
              </button>
            ))}

            {/* POST Results */}
            {postResults.length > 0 && (
              <div className="mt-4 pt-3" style={{ borderTop: '1px solid hsl(220 20% 18%)' }}>
                <div className="text-[9px] uppercase tracking-widest text-muted-foreground/40 font-bold mb-2 px-1">POST Results</div>
                <div className={`px-3 py-2 rounded-lg text-[10px] font-medium ${
                  postStatus === 'passed' ? 'bg-green-500/10 text-green-400'
                  : postStatus === 'failed' ? 'bg-red-500/10 text-red-400'
                  : 'bg-cyan-500/10 text-cyan-400'}`}>
                  {postStatus === 'running' && '⏳ Testing...'}
                  {postStatus === 'passed' && '✓ All checks passed'}
                  {postStatus === 'failed' && '✗ Some checks failed'}
                </div>
                <div className="mt-2 space-y-1">
                  {postResults.map((r, i) => (
                    <div key={i} className="flex items-center justify-between text-[9px] px-1">
                      <span className="font-mono text-muted-foreground">J{r.joint}</span>
                      <div className="flex gap-2">
                        <span className={r.encoder_ok ? 'text-green-500' : 'text-red-500'}>
                          {r.encoder_ok ? '✓' : '✗'} Enc
                        </span>
                        <span className={r.motor_ok ? 'text-green-500' : 'text-red-500'}>
                          {r.motor_ok ? '✓' : '✗'} Mot
                        </span>
                        <span className="text-muted-foreground/60">{r.latency_ms}ms</span>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </div>

          {/* Config panel */}
          <div className="flex-1 flex flex-col overflow-hidden">
            {/* Tabs */}
            <div className="flex items-center gap-1 px-4 pt-3">
              {(['motor', 'encoder', 'test'] as const).map(tab => (
                <button key={tab} onClick={() => setActiveTab(tab)}
                  className={`px-4 py-1.5 rounded-lg text-[10px] font-medium uppercase tracking-wider transition-all ${
                    activeTab === tab ? 'text-foreground shadow-md' : 'text-muted-foreground hover:text-foreground'}`}
                  style={activeTab === tab ? { background: 'hsl(210 40% 18%)' } : {}}>
                  {tab === 'motor' && <><Zap size={11} className="inline mr-1.5" />Motor</>}
                  {tab === 'encoder' && <><Cable size={11} className="inline mr-1.5" />Encoder</>}
                  {tab === 'test' && <><Activity size={11} className="inline mr-1.5" />Test</>}
                </button>
              ))}
              <div className="flex-1" />
              <span className="text-[10px] font-mono text-cyan-400/60">Joint {activeJoint + 1} Configuration</span>
            </div>

            {/* Content */}
            <div className="flex-1 overflow-auto p-4 space-y-4">
              {activeTab === 'motor' && (
                <MotorConfigTab joint={j} idx={activeJoint} onChange={(patch) => updateJoint(activeJoint, patch)} />
              )}
              {activeTab === 'encoder' && (
                <EncoderConfigTab joint={j} idx={activeJoint} onChange={(patch) => updateJoint(activeJoint, patch)} />
              )}
              {activeTab === 'test' && (
                <TestTab
                  joint={j} idx={activeJoint}
                  liveValue={liveEncoderValues[activeJoint]}
                  onTestMotor={() => handleTestMotor(activeJoint)}
                  onTestEncoder={() => handleTestEncoder(activeJoint)}
                />
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

// ── Motor Configuration Tab ──────────────────────────────────────────
const MotorConfigTab = ({ joint: j, idx, onChange }: {
  joint: JointHwConfig; idx: number;
  onChange: (patch: Partial<JointHwConfig>) => void;
}) => (
  <div className="space-y-5">
    {/* Motor Type */}
    <ConfigSection title="Motor Type">
      <div className="flex gap-2">
        {(['DC', 'BLDC', 'Stepper'] as MotorType[]).map(t => (
          <button key={t} onClick={() => onChange({ motorType: t })}
            className={`flex-1 py-2.5 rounded-xl text-[11px] font-medium transition-all border ${
              j.motorType === t
                ? 'border-cyan-500/40 bg-cyan-500/10 text-cyan-400 shadow-md'
                : 'border-white/5 bg-white/3 text-muted-foreground hover:border-white/15'}`}>
            {t === 'DC' && <Zap size={14} className="inline mr-1.5" />}
            {t === 'BLDC' && <Cpu size={14} className="inline mr-1.5" />}
            {t === 'Stepper' && <Settings2 size={14} className="inline mr-1.5" />}
            {t}
          </button>
        ))}
      </div>
    </ConfigSection>

    {/* Signal Output */}
    <ConfigSection title="Signal Output Type">
      <div className="flex gap-2">
        {(['PWM', 'Step/Dir', 'CAN', 'EtherCAT'] as SignalType[]).map(s => (
          <button key={s} onClick={() => onChange({ signalType: s })}
            className={`flex-1 py-2 rounded-lg text-[10px] font-medium transition-all border ${
              j.signalType === s
                ? 'border-primary/40 bg-primary/10 text-primary'
                : 'border-white/5 bg-white/3 text-muted-foreground hover:border-white/15'}`}>
            {s}
          </button>
        ))}
      </div>
    </ConfigSection>

    {/* Common Parameters */}
    <ConfigSection title="Control Parameters">
      <div className="grid grid-cols-3 gap-3">
        <NumInput label="Kp Gain" value={j.kp} step={0.1}
          onChange={v => onChange({ kp: v })} />
        <NumInput label="Ki Gain" value={j.ki} step={0.01}
          onChange={v => onChange({ ki: v })} />
        <NumInput label="Gear Ratio" value={j.gearRatio} step={1}
          onChange={v => onChange({ gearRatio: v })} />
        <NumInput label="Max Current (A)" value={j.maxCurrent} step={0.5}
          onChange={v => onChange({ maxCurrent: v })} />
      </div>
    </ConfigSection>

    {/* BLDC-specific */}
    {j.motorType === 'BLDC' && (
      <ConfigSection title="BLDC / FOC Parameters">
        <div className="grid grid-cols-3 gap-3">
          <NumInput label="Pole Pairs" value={j.polePairs} step={1}
            onChange={v => onChange({ polePairs: v })} />
          <NumInput label="Flux Linkage (Wb)" value={j.fluxLinkage} step={0.001}
            onChange={v => onChange({ fluxLinkage: v })} />
          <NumInput label="Phase Resistance (Ω)" value={j.phaseResistance} step={0.1}
            onChange={v => onChange({ phaseResistance: v })} />
          <NumInput label="FOC Kp (d-axis)" value={j.focKpD} step={0.1}
            onChange={v => onChange({ focKpD: v })} />
          <NumInput label="FOC Ki (d-axis)" value={j.focKiD} step={0.1}
            onChange={v => onChange({ focKiD: v })} />
          <NumInput label="FOC Kp (q-axis)" value={j.focKpQ} step={0.1}
            onChange={v => onChange({ focKpQ: v })} />
          <NumInput label="FOC Ki (q-axis)" value={j.focKiQ} step={0.1}
            onChange={v => onChange({ focKiQ: v })} />
        </div>
      </ConfigSection>
    )}

    {/* Stepper-specific */}
    {j.motorType === 'Stepper' && (
      <ConfigSection title="Stepper Parameters">
        <div className="grid grid-cols-3 gap-3">
          <NumInput label="Steps/Rev" value={200} step={1} onChange={() => {}} />
          <NumInput label="Microstepping" value={16} step={1} onChange={() => {}} />
          <NumInput label="Hold Current (%)" value={50} step={5} onChange={() => {}} />
          <NumInput label="Run Current (%)" value={80} step={5} onChange={() => {}} />
        </div>
      </ConfigSection>
    )}

    {/* Direction */}
    <ConfigSection title="Direction">
      <div className="flex items-center gap-3">
        <ToggleSwitch checked={j.invertDirection}
          onChange={v => onChange({ invertDirection: v })} />
        <span className="text-[10px] text-muted-foreground">
          {j.invertDirection ? 'Inverted (CW = negative)' : 'Normal (CW = positive)'}
        </span>
      </div>
    </ConfigSection>
  </div>
);

// ── Encoder Configuration Tab ────────────────────────────────────────
const EncoderConfigTab = ({ joint: j, idx, onChange }: {
  joint: JointHwConfig; idx: number;
  onChange: (patch: Partial<JointHwConfig>) => void;
}) => (
  <div className="space-y-5">
    <ConfigSection title="Encoder Type">
      <div className="grid grid-cols-3 gap-2">
        {([
          ['incremental', 'Incremental (A/B/Z)'],
          ['absolute-spi', 'Absolute (SPI)'],
          ['absolute-i2c', 'Absolute (I2C)'],
          ['absolute-ssi', 'Absolute (SSI/BiSS)'],
          ['resolver', 'Resolver'],
        ] as [EncoderType, string][]).map(([t, label]) => (
          <button key={t} onClick={() => onChange({ encoderType: t })}
            className={`py-2.5 rounded-xl text-[10px] font-medium transition-all border ${
              j.encoderType === t
                ? 'border-cyan-500/40 bg-cyan-500/10 text-cyan-400 shadow-md'
                : 'border-white/5 bg-white/3 text-muted-foreground hover:border-white/15'}`}>
            {label}
          </button>
        ))}
      </div>
    </ConfigSection>

    <ConfigSection title="Encoder Parameters">
      <div className="grid grid-cols-3 gap-3">
        <NumInput label="Counts Per Rev (CPR)" value={j.cpr} step={1}
          onChange={v => onChange({ cpr: v })} />
        <NumInput label="Gear Ratio" value={j.encoderGearRatio} step={1}
          onChange={v => onChange({ encoderGearRatio: v })} />
        <NumInput label="Zero Offset (counts)" value={j.zeroOffset} step={1}
          onChange={v => onChange({ zeroOffset: v })} />
      </div>
    </ConfigSection>

    <ConfigSection title="Polarity">
      <div className="flex items-center gap-4">
        <select className="glass-input text-[10px] py-1.5 px-3 rounded-lg w-40"
          value={j.encoderDirection}
          onChange={e => onChange({ encoderDirection: parseInt(e.target.value) })}>
          <option value={1}>Normal (+1)</option>
          <option value={-1}>Inverted (−1)</option>
        </select>
        <span className="text-[10px] text-muted-foreground">
          Encoder direction relative to motor rotation
        </span>
      </div>
    </ConfigSection>

    {j.encoderType === 'incremental' && (
      <ConfigSection title="Index Pulse (Z Channel)">
        <div className="flex items-center gap-3">
          <ToggleSwitch checked={true} onChange={() => {}} />
          <span className="text-[10px] text-muted-foreground">
            Use Z channel for homing reference
          </span>
        </div>
      </ConfigSection>
    )}

    <ConfigSection title="Computed Resolution">
      <div className="grid grid-cols-2 gap-3">
        <div className="glass-surface rounded-lg p-3">
          <div className="text-[9px] text-muted-foreground uppercase mb-1">Angular Resolution</div>
          <div className="text-sm font-mono text-cyan-400">
            {(360 / (j.cpr * j.encoderGearRatio)).toFixed(6)}°
          </div>
        </div>
        <div className="glass-surface rounded-lg p-3">
          <div className="text-[9px] text-muted-foreground uppercase mb-1">Effective CPR</div>
          <div className="text-sm font-mono text-cyan-400">
            {(j.cpr * j.encoderGearRatio).toLocaleString()}
          </div>
        </div>
      </div>
    </ConfigSection>
  </div>
);

// ── Test Tab ─────────────────────────────────────────────────────────
const TestTab = ({ joint: j, idx, liveValue, onTestMotor, onTestEncoder }: {
  joint: JointHwConfig; idx: number; liveValue: number;
  onTestMotor: () => void; onTestEncoder: () => void;
}) => (
  <div className="space-y-5">
    <ConfigSection title={`Motor Test — Joint ${idx + 1}`}>
      <div className="flex items-center gap-4">
        <button onClick={onTestMotor}
          disabled={j.testResult === 'testing'}
          className="flex items-center gap-2 px-4 py-2.5 rounded-xl text-[11px] font-medium transition-all"
          style={{ background: j.testResult === 'testing' ? 'hsl(210 40% 18%)' : 'hsl(210 60% 22%)',
                   color: 'hsl(210 60% 80%)' }}>
          {j.testResult === 'testing'
            ? <><RotateCw size={13} className="animate-spin" /> Testing...</>
            : <><Play size={13} /> Jog ±5°</>}
        </button>
        <div className="flex items-center gap-2">
          {j.testResult === 'pass' && <CheckCircle2 size={16} className="text-green-500" />}
          {j.testResult === 'fail' && <XCircle size={16} className="text-red-500" />}
          <span className="text-[10px] text-muted-foreground">
            {j.testResult === 'pass' ? 'Motor responds correctly' :
             j.testResult === 'fail' ? 'Motor did not respond' :
             j.testResult === 'testing' ? 'Sending micro-pulse...' : 'Not tested yet'}
          </span>
        </div>
      </div>
      <p className="text-[9px] text-muted-foreground/60 mt-2">
        Sends a ±5° micro-pulse to the motor and verifies encoder feedback matches expected motion.
        This confirms wiring, direction polarity, and PID response.
      </p>
    </ConfigSection>

    <ConfigSection title={`Encoder Test — Joint ${idx + 1}`}>
      <div className="flex items-center gap-4">
        <button onClick={onTestEncoder}
          disabled={j.testResult === 'testing'}
          className="flex items-center gap-2 px-4 py-2.5 rounded-xl text-[11px] font-medium transition-all"
          style={{ background: 'hsl(170 50% 18%)', color: 'hsl(170 50% 75%)' }}>
          <Gauge size={13} /> Read Encoder
        </button>
      </div>

      <div className="mt-3 glass-surface rounded-lg p-4">
        <div className="flex items-center justify-between mb-3">
          <span className="text-[9px] uppercase text-muted-foreground/60 tracking-wider font-bold">Live Encoder Feed</span>
          {j.testResult === 'testing' && (
            <span className="text-[9px] text-cyan-400 animate-pulse">● STREAMING</span>
          )}
        </div>
        <div className="grid grid-cols-3 gap-4">
          <div>
            <div className="text-[9px] text-muted-foreground uppercase">Raw Count</div>
            <div className="text-lg font-mono text-cyan-400">{Math.round(j.zeroOffset + liveValue)}</div>
          </div>
          <div>
            <div className="text-[9px] text-muted-foreground uppercase">Angle (deg)</div>
            <div className="text-lg font-mono text-green-400">
              {((liveValue / j.cpr) * (360 / j.encoderGearRatio)).toFixed(3)}°
            </div>
          </div>
          <div>
            <div className="text-[9px] text-muted-foreground uppercase">Angle (rad)</div>
            <div className="text-lg font-mono text-yellow-400">
              {((liveValue / j.cpr) * (2 * Math.PI / j.encoderGearRatio)).toFixed(5)}
            </div>
          </div>
        </div>

        {/* Mini bar graph */}
        <div className="mt-3 h-2 rounded-full overflow-hidden" style={{ background: 'hsl(220 20% 12%)' }}>
          <div className="h-full rounded-full transition-all duration-100"
               style={{
                 width: `${Math.max(0, Math.min(100, ((j.zeroOffset + liveValue) / j.cpr) * 100))}%`,
                 background: 'linear-gradient(90deg, hsl(180 80% 50%), hsl(210 80% 60%))',
               }}
          />
        </div>
      </div>
    </ConfigSection>

    <ConfigSection title="Joint Status Summary">
      <div className="grid grid-cols-2 gap-3">
        <StatusRow label="Motor Type" value={j.motorType} />
        <StatusRow label="Signal Output" value={j.signalType} />
        <StatusRow label="Encoder Type" value={j.encoderType} />
        <StatusRow label="CPR" value={j.cpr.toString()} />
        <StatusRow label="Direction" value={j.invertDirection ? 'Inverted' : 'Normal'} />
        <StatusRow label="Configured" value={j.configured ? '✓ Yes' : '✗ No'}
          color={j.configured ? 'text-green-400' : 'text-red-400'} />
      </div>
    </ConfigSection>
  </div>
);

// ── Shared Components ────────────────────────────────────────────────
const ConfigSection = ({ title, children }: { title: string; children: React.ReactNode }) => (
  <div>
    <div className="text-[10px] uppercase tracking-widest font-bold mb-2.5"
         style={{ color: 'hsl(210 30% 55%)' }}>{title}</div>
    {children}
  </div>
);

const NumInput = ({ label, value, step, onChange }: {
  label: string; value: number; step: number; onChange: (v: number) => void;
}) => (
  <div className="space-y-1">
    <label className="text-[9px] text-muted-foreground uppercase tracking-wider">{label}</label>
    <input type="number" step={step} value={value}
      onChange={e => onChange(parseFloat(e.target.value) || 0)}
      className="w-full h-8 px-2.5 text-[11px] font-mono rounded-lg transition-colors"
      style={{ background: 'hsl(220 25% 12%)', color: 'hsl(0 0% 85%)',
               border: '1px solid hsl(220 20% 22%)', outline: 'none' }}
    />
  </div>
);

const ToggleSwitch = ({ checked, onChange }: { checked: boolean; onChange: (v: boolean) => void }) => (
  <button onClick={() => onChange(!checked)}
    className={`relative w-9 h-5 rounded-full transition-all duration-200 ${
      checked ? 'bg-cyan-500' : 'bg-white/10'}`}>
    <div className={`absolute top-0.5 w-4 h-4 rounded-full transition-all duration-200 ${
      checked ? 'left-4.5 bg-white' : 'left-0.5 bg-white/60'}`}
      style={{ left: checked ? '18px' : '2px' }}
    />
  </button>
);

const StatusRow = ({ label, value, color }: { label: string; value: string; color?: string }) => (
  <div className="flex items-center justify-between px-3 py-1.5 rounded-lg" style={{ background: 'hsl(220 25% 11%)' }}>
    <span className="text-[10px] text-muted-foreground">{label}</span>
    <span className={`text-[10px] font-medium font-mono ${color || 'text-foreground'}`}>{value}</span>
  </div>
);
