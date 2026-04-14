import React, { useState } from 'react';
import { useAppState, SlotId, ToolId } from '@/store/AppState';
import { ChevronDown, Maximize2, MoreHorizontal } from 'lucide-react';

// Import all panels
import { ProgramTreePanel } from '../ProgramTreePanel';
import { BlockLibraryPanel } from '../BlockLibraryPanel';
import { MotionPlanningPanel } from '../MotionPlanningPanel';
import { IOPanel } from '../IOPanel';
import { DiagnosticsPanel } from '../DiagnosticsPanel';
import { RobotConfigPanel } from '../RobotConfigPanel';
import { ProjectManagerPanel } from '../ProjectManagerPanel';
import { ROS2Panel } from '../ROS2Panel';
import { SceneOutlinerPanel } from '../SceneOutlinerPanel';
import { ConsolePanel } from '../ConsolePanel';
import { PropertiesPanel } from '../PropertiesPanel';

// !! IMPORTANT: ToolRegistry is a FUNCTION map, not static JSX nodes.
// Static JSX like `program: <ProgramTreePanel />` creates one instance at module load
// and shares state between all slots. Functions ensure fresh renders per slot.
const ToolRegistry: Record<ToolId, () => React.ReactNode> = {
  program:     () => <ProgramTreePanel />,
  blocks:      () => <BlockLibraryPanel />,
  scene:       () => <SceneOutlinerPanel />,
  motion:      () => <MotionPlanningPanel />,
  io:          () => <IOPanel />,
  diagnostics: () => <DiagnosticsPanel />,
  config:      () => <RobotConfigPanel />,
  project:     () => <ProjectManagerPanel />,
  ros2:        () => <ROS2Panel />,
  console:     () => <ConsolePanel />,
  settings:    () => <IOPanel />,
  fullscreen3d: () => null,
};

// Tool labels for the switcher dropdown
const TOOL_OPTIONS: { id: ToolId; label: string; icon: string }[] = [
  { id: 'program',     label: 'Program Tree',    icon: '</>' },
  { id: 'blocks',      label: 'Block Library',   icon: '🧩' },
  { id: 'scene',       label: 'Scene Outliner',  icon: '🗂' },
  { id: 'io',          label: 'I/O Signals',     icon: '📡' },
  { id: 'console',     label: 'Console',         icon: '🖥' },
  { id: 'motion',      label: 'Motion Planning', icon: '✢' },
  { id: 'diagnostics', label: 'Diagnostics',     icon: '🔍' },
  { id: 'ros2',        label: 'ROS 2 Bridge',    icon: '🔗' },
  { id: 'project',     label: 'Project',         icon: '📁' },
  { id: 'config',      label: 'Robot Config',    icon: '⚙' },
];

interface LayoutSlotProps {
  id: SlotId;
}

export const LayoutSlot: React.FC<LayoutSlotProps> = ({ id }) => {
  const { layoutSlots, setLayoutSlot } = useAppState();
  const activeTool = layoutSlots[id] ?? 'program';
  const [dropdownOpen, setDropdownOpen] = useState(false);

  const activeOption = TOOL_OPTIONS.find(t => t.id === activeTool);
  const ContentComponent = ToolRegistry[activeTool];

  return (
    <div className="flex flex-col h-full w-full bg-[hsl(var(--panel-bg))] overflow-hidden">
      {/* Slot Tab Header */}
      <div
        className="flex items-center h-8 px-2 gap-1 border-b border-white/5 select-none group shrink-0"
        style={{ background: 'rgba(0,0,0,0.3)' }}
      >
        {/* Tool switcher button */}
        <div className="relative flex-1">
          <button
            className="flex items-center gap-1.5 px-2 py-1 rounded text-[10px] font-semibold text-zinc-400 hover:text-zinc-100 hover:bg-white/5 transition-colors cursor-pointer w-full"
            onClick={() => setDropdownOpen(!dropdownOpen)}
          >
            <span className="text-[9px] text-zinc-600 uppercase tracking-widest">{id}:</span>
            <span className="text-[10px]">{activeOption?.icon}</span>
            <span className="flex-1 text-left">{activeOption?.label ?? activeTool}</span>
            <ChevronDown size={10} className="opacity-50 shrink-0" />
          </button>

          {/* Dropdown menu */}
          {dropdownOpen && (
            <>
              <div className="fixed inset-0 z-40" onClick={() => setDropdownOpen(false)} />
              <div
                className="absolute top-full left-0 mt-1 z-50 py-1 rounded-xl shadow-2xl border border-white/10 min-w-[170px]"
                style={{ background: 'var(--panel-bg, #1a1e27)', backgroundColor: '#1a1e27' }}
              >
                <div className="px-3 py-1 text-[9px] text-zinc-600 uppercase tracking-widest mb-0.5">
                  Switch Panel
                </div>
                {TOOL_OPTIONS.map(opt => (
                  <button
                    key={opt.id}
                    className={`w-full flex items-center gap-2 px-3 py-1.5 text-[11px] transition-colors ${
                      opt.id === activeTool
                        ? 'text-primary bg-primary/10'
                        : 'text-zinc-300 hover:bg-white/5'
                    }`}
                    onClick={() => {
                      setLayoutSlot(id, opt.id);
                      setDropdownOpen(false);
                    }}
                  >
                    <span className="text-[12px] shrink-0">{opt.icon}</span>
                    {opt.label}
                    {opt.id === activeTool && (
                      <span className="ml-auto text-[9px] text-primary/60">active</span>
                    )}
                  </button>
                ))}
              </div>
            </>
          )}
        </div>

        {/* Utility buttons — appear on hover */}
        <div className="flex items-center gap-0.5 opacity-0 group-hover:opacity-100 transition-opacity shrink-0">
          <button className="p-1 hover:bg-white/10 rounded text-zinc-600 hover:text-zinc-300 transition-colors" title="Maximize">
            <Maximize2 size={9} />
          </button>
          <button className="p-1 hover:bg-white/10 rounded text-zinc-600 hover:text-zinc-300 transition-colors" title="More options">
            <MoreHorizontal size={9} />
          </button>
        </div>
      </div>

      {/* Slot Content — rendered fresh each time tool changes */}
      <div className="flex-1 overflow-hidden relative min-h-0">
        {ContentComponent ? ContentComponent() : null}
      </div>
    </div>
  );
};
