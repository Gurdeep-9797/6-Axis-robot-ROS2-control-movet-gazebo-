import React from 'react';
import { useAppState } from '@/store/AppState';
import { ProgramTreePanel } from './ProgramTreePanel';
import { BlockLibraryPanel } from './BlockLibraryPanel';
import { MotionPlanningPanel } from './MotionPlanningPanel';
import { IOPanel } from './IOPanel';
import { DiagnosticsPanel } from './DiagnosticsPanel';
import { RobotConfigPanel } from './RobotConfigPanel';
import { SettingsPanel } from './SettingsPanel';
import { ProjectManagerPanel } from './ProjectManagerPanel';
import { ROS2Panel } from './ROS2Panel';
import { SceneOutlinerPanel } from './SceneOutlinerPanel';

export const LeftPanel = () => {
  const { activePanel } = useAppState();

  const panels: Record<string, React.ReactNode> = {
    program: <ProgramTreePanel />,
    blocks: <BlockLibraryPanel />,
    scene: <SceneOutlinerPanel />,
    motion: <MotionPlanningPanel />,
    io: <IOPanel />,
    diagnostics: <DiagnosticsPanel />,
    config: <RobotConfigPanel />,
    settings: <SettingsPanel />,
    project: <ProjectManagerPanel />,
    ros2: <ROS2Panel />,
  };

  return (
    <div className="h-full border-r border-border overflow-hidden" style={{ width: 260, background: 'hsl(var(--panel-bg))' }}>
      {panels[activePanel] || panels.program}
    </div>
  );
};
