import React from 'react';
import { useAppState, ToolId } from '@/store/AppState';
import { PropertiesPanel } from './PropertiesPanel';
import { DiagnosticsPanel } from './DiagnosticsPanel';
import { IOPanel } from './IOPanel';
import { ROS2Panel } from './ROS2Panel';
import { Settings, Bug, Radio, Link2 } from 'lucide-react';

export const InspectorPanel = () => {
  const { activeInspectorTab, setActiveInspectorTab } = useAppState();

  const tabs: { id: string; label: string; icon: any }[] = [
    { id: 'properties', label: 'Properties', icon: Settings },
    { id: 'io', label: 'I/O', icon: Radio },
    { id: 'diagnostics', label: 'Debug', icon: Bug },
    { id: 'ros2', label: 'ROS 2', icon: Link2 },
  ];

  const currentTab = activeInspectorTab || 'properties';

  return (
    <div className="flex flex-col h-full w-full bg-[hsl(var(--panel-bg))] overflow-hidden">
      {/* Native Tab Bar (Reverting the dropdown change) */}
      <div className="flex items-center h-8 px-1 gap-1 border-b border-white/5 shrink-0 bg-black/20">
        {tabs.map(tab => (
          <button
            key={tab.id}
            className={`flex items-center gap-1.5 px-3 py-1 rounded-md text-[10px] font-semibold transition-all ${
              currentTab === tab.id
                ? 'bg-primary/20 text-primary shadow-sm'
                : 'text-muted-foreground hover:text-foreground hover:bg-white/5'
            }`}
            onClick={() => setActiveInspectorTab(tab.id as any)}
          >
            <tab.icon size={11} strokeWidth={2} />
            {tab.label}
          </button>
        ))}
      </div>

      <div className="flex-1 overflow-hidden relative min-h-0">
        {currentTab === 'properties' && <PropertiesPanel />}
        {currentTab === 'io' && <IOPanel />}
        {currentTab === 'diagnostics' && <DiagnosticsPanel />}
        {currentTab === 'ros2' && <ROS2Panel />}
      </div>
    </div>
  );
};
