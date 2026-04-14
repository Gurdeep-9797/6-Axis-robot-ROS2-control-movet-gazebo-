import React, { useEffect } from 'react';
import { AppStateProvider, useAppState, ToolId } from '@/store/AppState';
import { TopRibbon } from '@/components/robot/TopRibbon';
import { LeftSidebar } from '@/components/robot/LeftSidebar';
import { Viewport3D } from '@/components/robot/Viewport3D';
import { CodeEditor } from '@/components/robot/CodeEditor';
import { SettingsPage } from '@/components/robot/SettingsPage';
import { InspectorPanel } from '@/components/robot/InspectorPanel';
import { ProgramTreePanel } from '@/components/robot/ProgramTreePanel';
import { BlockLibraryPanel } from '@/components/robot/BlockLibraryPanel';
import { SceneOutlinerPanel } from '@/components/robot/SceneOutlinerPanel';
import { ProjectManagerPanel } from '@/components/robot/ProjectManagerPanel';
import { ConsolePanel } from '@/components/robot/ConsolePanel';
import { DiagnosticsPanel } from '@/components/robot/DiagnosticsPanel';
import { PanelRightClose, PanelRightOpen, Code2, Eye } from 'lucide-react';
import { Panel, PanelGroup, PanelResizeHandle } from 'react-resizable-panels';

// Tools that are hosted via LayoutSlot and can be mapped from activePanel
const PANEL_TO_TOOL: Record<string, ToolId> = {
  program:     'program',
  blocks:      'blocks',
  scene:       'scene',
  motion:      'motion',
  io:          'io',
  diagnostics: 'diagnostics',
  config:      'config',
  project:     'project',
  ros2:        'ros2',
};

const Resizer = ({ horizontal }: { horizontal?: boolean }) => (
  <PanelResizeHandle className={`relative flex items-center justify-center transition-colors group ${horizontal ? 'w-full h-1 cursor-row-resize' : 'h-full w-1 cursor-col-resize'}`}>
    <div className={`bg-border/30 group-hover:bg-primary/50 transition-colors ${horizontal ? 'h-px w-8' : 'w-px h-8'}`} />
  </PanelResizeHandle>
);

const WorkspaceContent = () => {
  const { 
    activePanel, setActivePanel, 
    rightPanelVisible, setRightPanelVisible, 
    codeEditorVisible, setCodeEditorVisible, 
    layoutSlots, setLayoutSlot,
  } = useAppState();

  useEffect(() => {
    const preventContextMenu = (e: MouseEvent) => {
      // Allow context menu only inside specific fields like inputs
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLTextAreaElement) return;
      e.preventDefault();
    };
    window.addEventListener('contextmenu', preventContextMenu, true);
    return () => window.removeEventListener('contextmenu', preventContextMenu, true);
  }, []);

  return (
    <div className="flex flex-col h-screen w-full relative overflow-hidden bg-background text-foreground select-none">
      {/* Settings is a full-page overlay */}
      {activePanel === 'settings' && (
        <SettingsPage onClose={() => setActivePanel('program')} />
      )}
      
      <TopRibbon />
      
      <div className="flex flex-1 overflow-hidden min-h-0">
        <LeftSidebar />
        
        <PanelGroup direction="horizontal" className="flex-1 overflow-hidden min-h-0">
          {/* Left explorer panel — hidden in fullscreen3d and settings modes */}
          {activePanel !== 'fullscreen3d' && activePanel !== 'settings' && (
            <>
              <Panel defaultSize={18} minSize={10} maxSize={32} id="explorer-panel">
                {activePanel === 'program' && <ProgramTreePanel />}
                {activePanel === 'blocks' && <BlockLibraryPanel />}
                {activePanel === 'scene' && <SceneOutlinerPanel />}
                {activePanel === 'project' && <ProjectManagerPanel />}
                {['motion', 'io', 'diagnostics', 'config', 'ros2'].includes(activePanel) && (
                  <div className="p-4 text-xs text-muted-foreground">Select a tool from the sidebar</div>
                )}
              </Panel>
              <Resizer />
            </>
          )}

          {/* Center: Code Editor + 3D Viewport + Bottom slot */}
          <Panel defaultSize={65} minSize={30} id="center-panel">
            <div className="flex flex-col h-full overflow-hidden">
              {/* Center tab bar */}
              <div className="flex items-center h-8 px-2 gap-1.5 shrink-0" style={{ background: 'rgba(0,0,0,0.3)', borderBottom: '1px solid rgba(255,255,255,0.05)' }}>
                <button
                  className={`flex items-center gap-1 px-2.5 py-1 text-[11px] font-semibold rounded-md transition-all duration-200 ${
                    codeEditorVisible ? 'text-primary bg-primary/15' : 'text-zinc-500 hover:bg-white/5'
                  }`}
                  onClick={() => setCodeEditorVisible(!codeEditorVisible)}>
                  <Code2 size={12} /> Code Editor
                </button>
                <button className="flex items-center gap-1 px-2.5 py-1 text-[11px] font-semibold text-primary bg-primary/15 rounded-md cursor-default">
                  <Eye size={12} /> 3D Viewport
                </button>
                <div className="flex-1" />
                <button
                  className={`flex items-center gap-1 px-2.5 py-1 text-[11px] font-semibold rounded-md transition-all duration-200 ${
                    rightPanelVisible ? 'text-primary bg-primary/15' : 'text-zinc-500 hover:bg-white/5'
                  }`}
                  onClick={() => setRightPanelVisible(!rightPanelVisible)}>
                  {rightPanelVisible ? <PanelRightClose size={12} /> : <PanelRightOpen size={12} />}
                  <span>Inspect</span>
                </button>
              </div>

              {/* Code Editor + 3D split (vertical) with bottom slot */}
              <PanelGroup direction="vertical" className="flex-1 overflow-hidden min-h-0">
                <Panel defaultSize={72} minSize={20} id="viewport-area">
                  <PanelGroup direction="horizontal" className="h-full">
                    {codeEditorVisible && activePanel !== 'fullscreen3d' && (
                      <>
                        <Panel defaultSize={40} minSize={20} id="code-editor-panel">
                          <CodeEditor />
                        </Panel>
                        <Resizer />
                      </>
                    )}
                    <Panel defaultSize={60} minSize={20} className="relative" id="viewport-panel">
                      <Viewport3D />
                    </Panel>
                  </PanelGroup>
                </Panel>
                
                <Resizer horizontal />
                <Panel defaultSize={28} minSize={10} maxSize={55} id="bottom-panel">
                  {layoutSlots.bottom === 'console' && <ConsolePanel />}
                  {layoutSlots.bottom === 'diagnostics' && <DiagnosticsPanel />}
                  {layoutSlots.bottom !== 'console' && layoutSlots.bottom !== 'diagnostics' && <ConsolePanel />}
                </Panel>
              </PanelGroup>
            </div>
          </Panel>

          {/* Right inspector panel */}
          {rightPanelVisible && activePanel !== 'fullscreen3d' && (
            <>
              <Resizer />
              <Panel defaultSize={17} minSize={14} maxSize={32} id="inspector-panel">
                <InspectorPanel />
              </Panel>
            </>
          )}
        </PanelGroup>
      </div>
    </div>
  );
};

const RobotWorkspace = () => (
  <AppStateProvider>
    <WorkspaceContent />
  </AppStateProvider>
);

export default RobotWorkspace;
