import React, { useState, useEffect, useRef } from 'react';
import { useAppState, IOPoint } from '@/store/AppState';
import { Radio, Zap, Terminal, RefreshCw, Plus, Trash2, AlertTriangle } from 'lucide-react';
import { backendConnector } from '@/services/BackendConnector';

// ── Helpers ──────────────────────────────────────────────────────────────

const SignalStatusDot = ({ active, pulse }: { active: boolean; pulse?: boolean }) => (
  <div className={`w-2 h-2 rounded-full flex-shrink-0 transition-all duration-200 ${
    active
      ? `bg-emerald-400 shadow-[0_0_6px_rgba(52,211,153,0.7)] ${pulse ? 'animate-pulse' : ''}`
      : 'bg-zinc-600'
  }`} />
);

const IORow = ({
  io,
  onToggle,
  onValueChange,
  onNameChange,
}: {
  io: IOPoint;
  onToggle: () => void;
  onValueChange: (v: number) => void;
  onNameChange: (n: string) => void;
}) => {
  const isDigital = io.type.startsWith('digital');
  const isOutput = io.type.endsWith('out');
  const val = io.value;

  return (
    <div className={`flex items-center gap-2 px-3 py-2 rounded-lg transition-all duration-150 border ${
      isDigital && val
        ? 'bg-emerald-950/30 border-emerald-800/30'
        : 'bg-black/20 border-white/5 hover:border-white/10'
    }`}>
      <SignalStatusDot active={Boolean(isDigital ? val : (val as number) > 0)} pulse={isOutput && Boolean(val)} />

      {/* Channel label */}
      <span className="text-[9px] font-mono text-zinc-500 w-8 shrink-0">
        {io.type === 'digital-out' ? 'DO' : io.type === 'digital-in' ? 'DI' : io.type === 'analog-in' ? 'AI' : 'AO'}
        {io.id.match(/\d+/)?.[0]}
      </span>

      {/* Name */}
      <input
        className="flex-1 bg-transparent text-[11px] font-medium focus:outline-none truncate"
        value={io.name}
        onChange={e => onNameChange(e.target.value)}
        readOnly={!isOutput}
      />

      {/* Value */}
      {isDigital ? (
        <button
          disabled={!isOutput}
          onClick={onToggle}
          title={isOutput ? 'Click to toggle' : 'Read-only input'}
          className={`relative w-9 h-5 rounded-full transition-all duration-300 flex items-center ${
            val
              ? 'bg-emerald-500 shadow-[0_0_8px_rgba(52,211,153,0.4)]'
              : 'bg-zinc-700'
          } ${isOutput ? 'cursor-pointer' : 'opacity-60 cursor-not-allowed'}`}
        >
          <div className={`w-3.5 h-3.5 rounded-full bg-white shadow-sm transition-all duration-300 absolute ${val ? 'left-[18px]' : 'left-[2px]'}`} />
        </button>
      ) : (
        <div className="flex items-center gap-1.5">
          <span className="text-[11px] font-mono text-cyan-400 w-10 text-right">{(val as number).toFixed(1)}</span>
          {isOutput
            ? <input
                type="range" min={0} max={io.type === 'analog-out' ? 100 : 10} step={0.1}
                value={val as number}
                onChange={e => onValueChange(Number(e.target.value))}
                className="w-16 h-1 accent-cyan-500 rounded-full"
              />
            : <div className="w-16 h-1.5 rounded-full overflow-hidden bg-black/40">
                <div
                  className="h-full rounded-full transition-all duration-500"
                  style={{
                    width: `${Math.min(100, (val as number) / 10 * 100)}%`,
                    background: 'linear-gradient(90deg, hsl(195 80% 40%), hsl(195 80% 65%))',
                    boxShadow: '0 0 4px hsl(195 80% 65% / 0.5)',
                  }}
                />
              </div>
          }
        </div>
      )}
    </div>
  );
};

// ── Main IOPanel ─────────────────────────────────────────────────────────

export const IOPanel = () => {
  const { ioPoints, toggleIO, setIOPoints, addConsoleEntry } = useAppState();
  const [filter, setFilter] = useState<'all' | 'digital-out' | 'digital-in' | 'analog-in' | 'analog-out'>('all');
  const [simIO, setSimIO] = useState(false);
  const simRef = useRef<number | null>(null);
  const isConnected = backendConnector.status === 'connected';

  const ioPointsRef = useRef(ioPoints);
  useEffect(() => { ioPointsRef.current = ioPoints; }, [ioPoints]);

  // Simulate live analog values when "Simulate I/O" is enabled
  useEffect(() => {
    if (!simIO) { if (simRef.current) clearInterval(simRef.current); return; }
    simRef.current = window.setInterval(() => {
      const updated = ioPointsRef.current.map(io => {
        if (io.type === 'analog-in') {
          const base = io.id === 'ai0' ? 40 : 22;
          return { ...io, value: parseFloat((base + (Math.random() - 0.5) * 8).toFixed(1)) };
        }
        return io;
      });
      setIOPoints(updated);
    }, 1200);
    return () => { if (simRef.current) clearInterval(simRef.current); };
  }, [simIO, setIOPoints]);

  const displayed = filter === 'all' ? ioPoints : ioPoints.filter(io => io.type === filter);

  const handleToggle = (id: string) => {
    toggleIO(id);
    const io = ioPoints.find(p => p.id === id);
    if (io) {
      const nextVal = !io.value;
      addConsoleEntry('info', `[IO] ${io.name} → ${nextVal ? 'HIGH' : 'LOW'}`);
      if (isConnected) {
        /* Send ROS std_msgs/Bool to /io/<signal_name> */
        backendConnector.publishIO(io.name, nextVal);
      }
    }
  };

  const handleAnalogChange = (id: string, v: number) => {
    setIOPoints(ioPoints.map(p => p.id === id ? { ...p, value: v } : p));
    const io = ioPoints.find(p => p.id === id);
    if (io && isConnected) {
      backendConnector.publishIO(io.name, v);
    }
  };

  const sections: { label: string; filter: IOPoint['type'] }[] = [
    { label: 'Digital Outputs', filter: 'digital-out' },
    { label: 'Digital Inputs', filter: 'digital-in' },
    { label: 'Analog Inputs', filter: 'analog-in' },
    { label: 'Analog Outputs', filter: 'analog-out' },
  ];

  const activeOuts = ioPoints.filter(io => io.type === 'digital-out' && io.value).length;
  const activeIns = ioPoints.filter(io => io.type === 'digital-in' && io.value).length;

  return (
    <div className="flex flex-col h-full select-none">
      {/* Header */}
      <div className="panel-header" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.05)' }}>
        <div className="flex items-center gap-2">
          <Radio size={13} className="text-primary" />
          I/O Signals
        </div>
        <div className="flex items-center gap-2 ml-auto">
          <div className={`px-1.5 py-0.5 rounded text-[9px] font-mono ${isConnected ? 'bg-emerald-900/40 text-emerald-400' : 'bg-zinc-800 text-zinc-500'}`}>
            {isConnected ? '● ROS' : '○ SIM'}
          </div>
          <button
            onClick={() => setSimIO(!simIO)}
            className={`glass-btn !px-1.5 !py-0.5 text-[9px] ${simIO ? 'active' : ''}`}
            title="Simulate live analog fluctuations"
          >
            <Zap size={10} className={simIO ? 'text-yellow-400' : ''} />
          </button>
          <button className="glass-btn !px-1.5 !py-0.5" title="Refresh from ROS">
            <RefreshCw size={10} />
          </button>
        </div>
      </div>

      {/* Status Bar */}
      <div className="flex items-center gap-4 px-3 py-1.5 text-[10px] text-zinc-500" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.04)', background: 'hsl(0 0% 5%)' }}>
        <span><span className="text-emerald-400 font-mono">{activeOuts}</span> DO active</span>
        <span><span className="text-cyan-400 font-mono">{activeIns}</span> DI high</span>
        <span className="ml-auto">{ioPoints.length} signals</span>
      </div>

      {/* Filter tabs */}
      <div className="flex" style={{ borderBottom: '1px solid hsl(0 0% 100% / 0.04)' }}>
        {[
          { id: 'all', label: 'All' },
          { id: 'digital-out', label: 'DO' },
          { id: 'digital-in', label: 'DI' },
          { id: 'analog-in', label: 'AI' },
          { id: 'analog-out', label: 'AO' },
        ].map(tab => (
          <button
            key={tab.id}
            onClick={() => setFilter(tab.id as any)}
            className={`flex-1 py-1.5 text-[10px] font-semibold transition-all ${
              filter === tab.id ? 'text-primary border-b-2 border-primary' : 'text-zinc-500 hover:text-zinc-300'
            }`}
          >
            {tab.label}
          </button>
        ))}
      </div>

      {/* Signal list */}
      <div className="flex-1 overflow-auto p-2 space-y-1">
        {filter === 'all' ? (
          sections.map(sec => {
            const items = ioPoints.filter(io => io.type === sec.filter);
            if (!items.length) return null;
            return (
              <div key={sec.filter} className="mb-3">
                <div className="text-[9px] uppercase tracking-widest font-bold text-zinc-600 mb-1 px-1">{sec.label}</div>
                <div className="space-y-1">
                  {items.map(io => (
                    <IORow
                      key={io.id} io={io}
                      onToggle={() => handleToggle(io.id)}
                      onValueChange={v => handleAnalogChange(io.id, v)}
                      onNameChange={n => setIOPoints(ioPoints.map(p => p.id === io.id ? { ...p, name: n } : p))}
                    />
                  ))}
                </div>
              </div>
            );
          })
        ) : (
          displayed.map(io => (
            <IORow
              key={io.id} io={io}
              onToggle={() => handleToggle(io.id)}
              onValueChange={v => handleAnalogChange(io.id, v)}
              onNameChange={n => setIOPoints(ioPoints.map(p => p.id === io.id ? { ...p, name: n } : p))}
            />
          ))
        )}
      </div>

      {/* Add signal button */}
      <div className="p-2" style={{ borderTop: '1px solid hsl(0 0% 100% / 0.04)' }}>
        {!isConnected && (
          <div className="flex items-center gap-1.5 text-[10px] text-amber-500/70 mb-2">
            <AlertTriangle size={10} />
            Offline — signals are simulated
          </div>
        )}
        <button
          className="glass-btn w-full text-[10px] py-1.5"
          onClick={() => {
            const id = `do${ioPoints.filter(p => p.type === 'digital-out').length}`;
            setIOPoints([...ioPoints, { id, name: `DO_New_${ioPoints.length}`, type: 'digital-out', value: false }]);
          }}
        >
          <Plus size={10} className="inline mr-1" /> Add Signal
        </button>
      </div>
    </div>
  );
};
