import React from 'react';
import { useAppState } from '@/store/AppState';

export const SettingsPanel = () => {
  const { settingsTab, setSettingsTab } = useAppState();

  const tabs = ['graphics', 'limits', 'controls', 'physics', 'integration'];

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">Settings</div>
      {/* Tabs */}
      <div className="flex border-b border-border">
        {tabs.map(t => (
          <button key={t} className={`px-3 py-1.5 text-xs capitalize transition-colors ${settingsTab === t ? 'text-primary border-b-2 border-primary' : 'text-muted-foreground hover:text-foreground'}`}
            onClick={() => setSettingsTab(t)}>
            {t}
          </button>
        ))}
      </div>
      <div className="flex-1 overflow-auto p-3 space-y-3">
        {settingsTab === 'graphics' && (
          <>
            <SettingField label="Render API" value="DirectX 12" />
            <SettingField label="Target FPS" value="60" editable />
            <SettingSelect label="Quality" options={['Low', 'Medium', 'High', 'Ultra']} defaultVal="High" />
            <SettingToggle label="VSync" defaultVal={true} />
            <SettingToggle label="Anti-Aliasing" defaultVal={true} />
            <SettingToggle label="Shadows" defaultVal={true} />
            <SettingToggle label="Ambient Occlusion" defaultVal={false} />
          </>
        )}
        {settingsTab === 'limits' && (
          <>
            <SettingField label="Max TCP Speed" value="2000 mm/s" editable />
            <SettingField label="Max Joint Speed" value="250 °/s" editable />
            <SettingField label="Workspace Radius" value="2650 mm" />
            <SettingToggle label="Enforce Soft Limits" defaultVal={true} />
            <SettingToggle label="Collision Detection" defaultVal={true} />
          </>
        )}
        {settingsTab === 'controls' && (
          <>
            <SettingSelect label="Mouse Mode" options={['Orbit', 'Pan', 'Fly']} defaultVal="Orbit" />
            <SettingField label="Pan Speed" value="1.0" editable />
            <SettingField label="Zoom Speed" value="1.0" editable />
            <SettingToggle label="Invert Y-Axis" defaultVal={false} />
          </>
        )}
        {settingsTab === 'physics' && (
          <>
            <SettingField label="Gravity" value="9.81 m/s²" editable />
            <SettingField label="Friction Coefficient" value="0.5" editable />
            <SettingField label="Time Step" value="0.001 s" editable />
            <SettingSelect label="Solver" options={['Bullet', 'PhysX', 'ODE']} defaultVal="Bullet" />
          </>
        )}
        {settingsTab === 'integration' && (
          <>
            <SettingField label="SolidWorks API" value="Connected ✓" />
            <SettingField label="ROS 2 Bridge" value="Active" />
            <SettingField label="Controller IP" value="192.168.1.100" editable />
            <SettingField label="Controller Port" value="5000" editable />
            <SettingToggle label="Auto-sync CAD" defaultVal={true} />
          </>
        )}
      </div>
    </div>
  );
};

const SettingField = ({ label, value, editable }: { label: string; value: string; editable?: boolean }) => (
  <div className="property-field">
    <span className="property-label">{label}</span>
    {editable ? <input className="property-value w-24" defaultValue={value} /> : <span className="text-xs">{value}</span>}
  </div>
);

const SettingSelect = ({ label, options, defaultVal }: { label: string; options: string[]; defaultVal: string }) => (
  <div className="property-field">
    <span className="property-label">{label}</span>
    <select className="property-value w-24" defaultValue={defaultVal}>
      {options.map(o => <option key={o}>{o}</option>)}
    </select>
  </div>
);

const SettingToggle = ({ label, defaultVal }: { label: string; defaultVal: boolean }) => {
  const [on, setOn] = React.useState(defaultVal);
  return (
    <div className="property-field">
      <span className="property-label">{label}</span>
      <button className={`w-8 h-4 rounded-full transition-all relative ${on ? 'bg-status-ok' : 'bg-secondary'}`} onClick={() => setOn(!on)}>
        <div className={`absolute top-0.5 w-3 h-3 rounded-full bg-foreground transition-transform ${on ? 'translate-x-4' : 'translate-x-0.5'}`} />
      </button>
    </div>
  );
};
