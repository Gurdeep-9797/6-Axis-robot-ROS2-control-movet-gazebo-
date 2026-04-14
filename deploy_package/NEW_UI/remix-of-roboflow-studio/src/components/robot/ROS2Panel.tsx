import React, { useState } from 'react';
import { useAppState } from '@/store/AppState';
import { Wifi, WifiOff, Activity, Radio } from 'lucide-react';

interface ROS2Topic {
  name: string;
  type: string;
  direction: 'pub' | 'sub';
  rate: number;
  active: boolean;
}

const defaultTopics: ROS2Topic[] = [
  { name: '/joint_states', type: 'sensor_msgs/JointState', direction: 'pub', rate: 50, active: true },
  { name: '/tcp_pose', type: 'geometry_msgs/PoseStamped', direction: 'pub', rate: 50, active: true },
  { name: '/trajectory', type: 'trajectory_msgs/JointTrajectory', direction: 'pub', rate: 10, active: false },
  { name: '/cmd_joint', type: 'sensor_msgs/JointState', direction: 'sub', rate: 0, active: true },
  { name: '/cmd_pose', type: 'geometry_msgs/PoseStamped', direction: 'sub', rate: 0, active: true },
  { name: '/external_trigger', type: 'std_msgs/Bool', direction: 'sub', rate: 0, active: false },
];

export const ROS2Panel = () => {
  const { addConsoleEntry } = useAppState();
  const [connected, setConnected] = useState(true);
  const [namespace, setNamespace] = useState('/robot_1');
  const [publishRate, setPublishRate] = useState(50);
  const [topics, setTopics] = useState<ROS2Topic[]>(defaultTopics);

  const toggleConnection = () => {
    setConnected(!connected);
    addConsoleEntry(connected ? 'warning' : 'success',
      connected ? 'ROS 2 bridge disconnected' : 'ROS 2 bridge connected');
  };

  const toggleTopic = (name: string) => {
    setTopics(prev => prev.map(t => t.name === name ? { ...t, active: !t.active } : t));
  };

  return (
    <div className="flex flex-col h-full">
      <div className="panel-header">
        ROS 2 Bridge
        <button className={`transition-colors ${connected ? 'text-status-ok' : 'text-status-error'}`}
          onClick={toggleConnection} title={connected ? 'Disconnect' : 'Connect'}>
          {connected ? <Wifi size={14} /> : <WifiOff size={14} />}
        </button>
      </div>
      <div className="flex-1 overflow-auto p-3 space-y-4">
        <div className="glass-surface rounded-lg p-3">
          <div className="flex items-center gap-2 mb-2.5">
            <div className={`status-led ${connected ? 'on' : 'off'}`} />
            <span className="text-xs font-semibold">{connected ? 'Connected' : 'Disconnected'}</span>
          </div>
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-[10px] text-muted-foreground">Namespace</span>
              <input className="glass-input w-24 text-[10px]" value={namespace} onChange={e => setNamespace(e.target.value)} />
            </div>
            <div className="flex items-center justify-between">
              <span className="text-[10px] text-muted-foreground">Publish Rate</span>
              <div className="flex items-center gap-1">
                <input type="number" className="glass-input w-14 text-[10px]" value={publishRate} onChange={e => setPublishRate(Number(e.target.value))} />
                <span className="text-[10px] text-muted-foreground">Hz</span>
              </div>
            </div>
          </div>
        </div>

        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Published Topics</span>
          <div className="space-y-1 mt-1.5">
            {topics.filter(t => t.direction === 'pub').map(topic => (
              <div key={topic.name} className="flex items-center gap-2 py-2 px-2.5 rounded-lg hover:bg-accent/40 transition-all cursor-pointer"
                onClick={() => toggleTopic(topic.name)}>
                <Activity size={10} className={topic.active ? 'text-status-ok' : 'text-muted-foreground'} />
                <div className="flex-1 min-w-0">
                  <div className="text-[11px] font-mono truncate">{topic.name}</div>
                  <div className="text-[9px] text-muted-foreground truncate">{topic.type}</div>
                </div>
                <span className="text-[9px] text-muted-foreground">{topic.active ? `${topic.rate}Hz` : 'off'}</span>
              </div>
            ))}
          </div>
        </div>

        <div>
          <span className="text-[10px] text-muted-foreground uppercase tracking-wider font-semibold">Subscribed Topics</span>
          <div className="space-y-1 mt-1.5">
            {topics.filter(t => t.direction === 'sub').map(topic => (
              <div key={topic.name} className="flex items-center gap-2 py-2 px-2.5 rounded-lg hover:bg-accent/40 transition-all cursor-pointer"
                onClick={() => toggleTopic(topic.name)}>
                <Radio size={10} className={topic.active ? 'text-status-info' : 'text-muted-foreground'} />
                <div className="flex-1 min-w-0">
                  <div className="text-[11px] font-mono truncate">{topic.name}</div>
                  <div className="text-[9px] text-muted-foreground truncate">{topic.type}</div>
                </div>
                <span className="text-[9px] text-muted-foreground">{topic.active ? 'active' : 'off'}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};
