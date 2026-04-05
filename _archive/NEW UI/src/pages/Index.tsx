import React from 'react';
import { AppStateProvider } from '@/store/AppState';
import { TopRibbon } from '@/components/robot/TopRibbon';
import { LeftSidebar } from '@/components/robot/LeftSidebar';
import { LeftPanel } from '@/components/robot/LeftPanel';
import { Viewport3D } from '@/components/robot/Viewport3D';
import { CodeEditor } from '@/components/robot/CodeEditor';
import { PropertiesPanel } from '@/components/robot/PropertiesPanel';
import { ConsolePanel } from '@/components/robot/ConsolePanel';
import { useAppState } from '@/store/AppState';
import { PanelRightClose, PanelRightOpen, Code2, Eye } from 'lucide-react';

const WorkspaceContent = () => {
  const { rightPanelVisible, setRightPanelVisible, codeEditorVisible, setCodeEditorVisible } = useAppState();

  return (
    <div className="flex flex-col h-screen w-full">
      <TopRibbon />
      <div className="flex flex-1 overflow-hidden">
        <LeftSidebar />
        <LeftPanel />

        {/* Main content area */}
        <div className="flex flex-col flex-1 overflow-hidden">
          {/* Toolbar for center area */}
          <div className="flex items-center h-7 px-2 gap-2 border-b border-border" style={{ background: 'hsl(var(--panel-header))' }}>
            <button
              className={`flex items-center gap-1 px-2 py-0.5 text-[10px] rounded-sm transition-colors ${codeEditorVisible ? 'text-primary' : 'text-muted-foreground hover:text-foreground'}`}
              onClick={() => setCodeEditorVisible(!codeEditorVisible)}
            >
              <Code2 size={12} /> Code
            </button>
            <button className="flex items-center gap-1 px-2 py-0.5 text-[10px] text-primary rounded-sm">
              <Eye size={12} /> 3D View
            </button>
            <div className="flex-1" />
            <button
              className="flex items-center gap-1 px-2 py-0.5 text-[10px] text-muted-foreground hover:text-foreground rounded-sm transition-colors"
              onClick={() => setRightPanelVisible(!rightPanelVisible)}
            >
              {rightPanelVisible ? <PanelRightClose size={12} /> : <PanelRightOpen size={12} />}
              Properties
            </button>
          </div>

          {/* Split view */}
          <div className="flex flex-1 overflow-hidden">
            <div className="flex flex-1 overflow-hidden">
              {codeEditorVisible && (
                <div className="border-r border-border" style={{ width: '40%', minWidth: 300 }}>
                  <CodeEditor />
                </div>
              )}
              <div className="flex-1">
                <Viewport3D />
              </div>
            </div>
            {rightPanelVisible && (
              <div className="border-l border-border animate-slide-in-right" style={{ width: 260, background: 'hsl(var(--panel-bg))' }}>
                <PropertiesPanel />
              </div>
            )}
          </div>

          <ConsolePanel />
        </div>
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
