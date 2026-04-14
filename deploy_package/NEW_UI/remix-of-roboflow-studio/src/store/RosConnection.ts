import * as ROSLIB from 'roslib';
import { create } from 'zustand';

export type RosStatus = 'connected' | 'error' | 'closed';

type StatusCallback = (status: RosStatus) => void;
type JointCallback = (angles: number[]) => void;

class RosConnection {
  public ros: ROSLIB.Ros;
  private trajectoryTopic!: ROSLIB.Topic;
  private jointStateSub!: ROSLIB.Topic;
  private trackingErrorSub!: ROSLIB.Topic;
  private healthSub!: ROSLIB.Topic;
  private ikService!: ROSLIB.Service;
  private healthCheckService!: ROSLIB.Service;
  
  public status: RosStatus = 'closed';
  private statusListeners: StatusCallback[] = [];
  private jointListeners: JointCallback[] = [];
  private url: string;

  constructor(url: string = 'ws://localhost:9090') {
    this.url = url;
    this.ros = new ROSLIB.Ros({});
    this.setupListeners();
    this.connect();
  }

  private setupListeners() {
    this.ros.on('connection', () => {
      console.log('[ROS] Connected to ' + this.url);
      this.status = 'connected';
      this.notifyStatus();
      this.setupTopicsAndServices();
    });

    this.ros.on('error', (error: any) => {
      console.warn('[ROS] Connection error:', error);
      this.status = 'error';
      this.notifyStatus();
    });

    this.ros.on('close', () => {
      console.log('[ROS] Connection closed. Retrying in 3s...');
      this.status = 'closed';
      this.notifyStatus();
      setTimeout(() => this.connect(), 3000);
    });
  }

  private connect() {
    try {
      this.ros.connect(this.url);
    } catch (e) {
      console.warn('[ROS] connect() threw:', e);
    }
  }

  private setupTopicsAndServices() {
    this.trajectoryTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/planned_trajectory',
      messageType: 'trajectory_msgs/JointTrajectory'
    });

    this.jointStateSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/joint_states',
      messageType: 'sensor_msgs/JointState'
    });

    this.ikService = new ROSLIB.Service({
      ros: this.ros,
      name: '/compute_ik',
      serviceType: 'moveit_msgs/srv/GetPositionIK'
    });

    this.healthCheckService = new ROSLIB.Service({
      ros: this.ros,
      name: '/roboforge/health_check',
      serviceType: 'std_srvs/srv/Trigger'
    });

    this.trackingErrorSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/tracking_error',
      messageType: 'std_msgs/String'
    });

    this.healthSub = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/health_status',
      messageType: 'std_msgs/String'
    });

    this.jointStateSub.subscribe((message: any) => {
      if (message.position && message.position.length >= 6) {
        const angles = Array.from(message.position).slice(0, 6) as number[];
        this.jointListeners.forEach(cb => cb(angles));
        useRobotStateStore.getState().setJointPos(angles);
      }
    });

    this.trackingErrorSub.subscribe((message: any) => {
      try {
        const data = JSON.parse(message.data);
        useRobotStateStore.getState().setTrackingError(data);
      } catch (e) {
        console.error('[ROS] Failed to parse tracking error:', e);
      }
    });

    this.healthSub.subscribe((message: any) => {
      try {
        const data = JSON.parse(message.data);
        useRobotStateStore.getState().setHealthResults(data);
      } catch (e) {
        console.error('[ROS] Failed to parse health status:', e);
      }
    });
  }

  public computeIK(x: number, y: number, z: number, rx: number, ry: number, rz: number, rw: number, currentJoints: number[]): Promise<number[] | null> {
    return new Promise((resolve) => {
      if (this.status !== 'connected') {
        resolve(null);
        return;
      }
      const request = {
        ik_request: {
          group_name: 'manipulator',
          robot_state: {
            joint_state: {
              name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
              position: currentJoints
            }
          },
          avoid_collisions: true,
          pose_stamped: {
            header: { frame_id: 'base_link' },
            pose: {
              position: { x: x / 1000.0, y: y / 1000.0, z: z / 1000.0 },
              orientation: { x: rx, y: ry, z: rz, w: rw }
            }
          },
          timeout: { sec: 1, nanosec: 0 }
        }
      };

      this.ikService.callService(request, (result: any) => {
        if (result.error_code && result.error_code.val === 1) {
          const positions = result.solution.joint_state.position;
          const idx = result.solution.joint_state.name.indexOf('joint_1');
          resolve(idx !== -1 ? positions.slice(idx, idx + 6) : positions.slice(0, 6));
        } else {
          resolve(null);
        }
      }, () => resolve(null));
    });
  }

  public publishTrajectory(joints: number[], durationSec: number = 1.0) {
    if (this.status !== 'connected' || !this.trajectoryTopic) return;
    const trajectoryMsg = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'world' },
      joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
      points: [{
        positions: joints,
        velocities: [0, 0, 0, 0, 0, 0],
        accelerations: [0, 0, 0, 0, 0, 0],
        effort: [0, 0, 0, 0, 0, 0],
        time_from_start: {
          sec: Math.floor(durationSec),
          nanosec: Math.floor((durationSec - Math.floor(durationSec)) * 1e9)
        }
      }]
    };
    this.trajectoryTopic.publish(trajectoryMsg);
  }

  public onStatus(cb: StatusCallback) {
    this.statusListeners.push(cb);
    cb(this.status);
  }

  public offStatus(cb: StatusCallback) {
    this.statusListeners = this.statusListeners.filter(l => l !== cb);
  }

  public onJointState(cb: JointCallback) {
    this.jointListeners.push(cb);
  }

  public offJointState(cb: JointCallback) {
    this.jointListeners = this.jointListeners.filter(l => l !== cb);
  }

  private notifyStatus() {
    this.statusListeners.forEach(cb => cb(this.status));
  }
}

export const rosConnection = new RosConnection('ws://localhost:9090');

// ── Zustand Robot State Store ──────────────────────────────────────────

export interface TrackingErrorReport {
  type: string;
  per_joint_error_deg: number[];
  per_joint_error_pct: number[];
  tcp_error_mm: number;
  max_error_joint: string;
  max_error_deg: number;
  timestamp_ns: number;
}

export interface HealthCheckResult {
  name: string;
  ok: boolean;
  detail: string;
}

interface RobotStateStore {
  jointPos: number[];
  trackingError: TrackingErrorReport | null;
  healthResults: HealthCheckResult[];
  mode: 'edit' | 'simulate' | 'live';
  source: 'gazebo' | 'encoder';
  setJointPos: (pos: number[]) => void;
  setTrackingError: (err: TrackingErrorReport) => void;
  setHealthResults: (results: HealthCheckResult[]) => void;
  setMode: (mode: 'edit' | 'simulate' | 'live') => void;
  runHealthCheck: () => Promise<boolean>;
}

export const useRobotStateStore = create<RobotStateStore>((set) => ({
  jointPos: [0, 0, 0, 0, 0, 0],
  trackingError: null,
  healthResults: [],
  mode: 'simulate',
  source: 'gazebo',
  setJointPos: (pos) => set({ jointPos: pos }),
  setTrackingError: (err) => set({ trackingError: err }),
  setHealthResults: (results) => set({ healthResults: results }),
  setMode: (mode) => set({ mode, source: mode === 'live' ? 'encoder' : 'gazebo' }),
  runHealthCheck: async () => {
    return new Promise((resolve) => {
      const request = {};
      (rosConnection as any).healthCheckService.callService(request, (result: any) => {
        try {
          const results = JSON.parse(result.message);
          set({ healthResults: results });
          resolve(result.success);
        } catch(e) { resolve(false); }
      }, () => resolve(false));
    });
  }
}));
