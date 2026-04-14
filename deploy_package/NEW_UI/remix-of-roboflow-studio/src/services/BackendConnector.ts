/**
 * BackendConnector — Environment-aware connection routing (Section 2.5 of Bible)
 * 
 * Priority:
 *   1. Try ws://localhost:9090 (local Docker backend)
 *   2. Try user-configured tunnel URL from localStorage
 *   3. Fall back to offline mode (JS IK solvers only)
 */

import * as ROSLIB from 'roslib';
import { solveIK, IKMode } from '@/engine/IKSolver';

export type ConnectionMode = 'local' | 'tunnel' | 'offline';
export type ConnectionStatus = 'connected' | 'connecting' | 'disconnected';

type StatusCallback = (status: ConnectionStatus, mode: ConnectionMode) => void;
type JointCallback = (angles: number[]) => void;

const TUNNEL_URL_KEY = 'roboforge_tunnel_ws';

class BackendConnector {
  private ros: ROSLIB.Ros | null = null;
  private trajectoryTopic: ROSLIB.Topic | null = null;
  private jointStateSub: ROSLIB.Topic | null = null;
  private ikService: ROSLIB.Service | null = null;

  public mode: ConnectionMode = 'offline';
  public status: ConnectionStatus = 'disconnected';

  private statusListeners: StatusCallback[] = [];
  private jointListeners: JointCallback[] = [];

  constructor() {
    this.autoConnect();
  }

  /** Attempt connection in priority order */
  private async autoConnect() {
    this.status = 'connecting';
    this.notifyStatus();

    // Priority 1: localhost
    if (await this.tryConnect('ws://localhost:9090')) {
      this.mode = 'local';
      console.log('[Bridge] Connected to local Docker backend');
      return;
    }

    // Priority 2: tunnel URL from localStorage
    const tunnelUrl = typeof localStorage !== 'undefined'
      ? localStorage.getItem(TUNNEL_URL_KEY)
      : null;
    if (tunnelUrl && await this.tryConnect(tunnelUrl)) {
      this.mode = 'tunnel';
      console.log(`[Bridge] Connected via tunnel: ${tunnelUrl}`);
      return;
    }

    // Priority 3: offline
    this.mode = 'offline';
    this.status = 'disconnected';
    console.warn('[Bridge] No ROS 2 backend found — offline simulation mode');
    this.notifyStatus();
  }

  /** Try to connect to a WebSocket URL with a 3-second timeout */
  private tryConnect(url: string): Promise<boolean> {
    return new Promise((resolve) => {
      const ros = new ROSLIB.Ros({});
      const timeout = setTimeout(() => {
        try { ros.close(); } catch (_) { /* ignore */ }
        resolve(false);
      }, 3000);

      ros.on('connection', () => {
        clearTimeout(timeout);
        this.ros = ros;
        this.status = 'connected';
        this.setupTopicsAndServices();
        this.notifyStatus();

        // Handle future disconnects
        ros.on('close', () => {
          console.log('[Bridge] Connection lost. Falling back to offline mode.');
          this.status = 'disconnected';
          this.mode = 'offline';
          this.ros = null;
          this.notifyStatus();
          // Retry after 5s
          setTimeout(() => this.autoConnect(), 5000);
        });

        resolve(true);
      });

      ros.on('error', () => {
        clearTimeout(timeout);
        try { ros.close(); } catch (_) { /* ignore */ }
        resolve(false);
      });

      try {
        ros.connect(url);
      } catch (_) {
        clearTimeout(timeout);
        resolve(false);
      }
    });
  }

  private setupTopicsAndServices() {
    if (!this.ros) return;

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

    this.jointStateSub.subscribe((message: any) => {
      if (message.position && message.position.length >= 6) {
        const angles = Array.from(message.position).slice(0, 6) as number[];
        this.jointListeners.forEach(cb => cb(angles));
      }
    });
  }

  /**
   * Compute IK — routes according to ikMode setting (NEVER auto-falls-back).
   *
   * ROUTING RULES (enforced strictly — no silent fallback):
   *   ikMode === 'moveit'             → MoveIt /compute_ik service via rosbridge ONLY.
   *                                     If backend is offline or service fails → returns null (error shown in UI).
   *   ikMode === 'offline-analytical' → Local JS analytical solver ONLY. No ROS required.
   *   ikMode === 'offline-numerical'  → Local JS numerical DLS solver ONLY. No ROS required.
   *
   * @param x_mm, y_mm, z_mm  — Target position in millimetres (UI coordinate frame)
   * @param ikMode            — From Settings toggle, defaults to 'moveit'
   * @param currentJoints     — Current joint angles in radians (used as IK seed)
   */
  public async computeIK(
    x_mm: number, y_mm: number, z_mm: number,
    rx: number, ry: number, rz: number, rw: number,
    ikMode: IKMode,
    currentJoints: number[]
  ): Promise<{ joints: number[]; solver: string } | null> {

    // ── OFFLINE MODES: use local JS solver, no backend needed ─────────────────
    if (ikMode === 'offline-analytical' || ikMode === 'offline-numerical') {
      const result = solveIK(
        { position: { x: x_mm, y: y_mm, z: z_mm }, orientation: { qw: rw, qx: rx, qy: ry, qz: rz } },
        ikMode,
        currentJoints
      );
      if (result.success) {
        console.log(`[IK] ${ikMode} → solved (error: ${result.error.toFixed(2)}mm)`);
        return { joints: result.joints, solver: ikMode };
      }
      console.error(`[IK] ${ikMode} failed — target unreachable (error: ${result.error.toFixed(2)}mm)`);
      return null;
    }

    // ── MOVEIT MODE (default): MoveIt /compute_ik ONLY ──────────────────────
    // If backend not connected, fail immediately — do NOT fall back to JS solver.
    if (this.mode === 'offline' || !this.ikService || !this.ros) {
      console.error(
        '[IK] Mode is "moveit" but no backend is connected. ' +
        'Start the Docker stack (docker-compose up -d) or switch to an Offline IK mode in Settings → Motion.'
      );
      return null;
    }

    try {
      const joints = await this.callMoveItIK(x_mm, y_mm, z_mm, rx, ry, rz, rw, currentJoints);
      if (joints) {
        console.log('[IK] MoveIt /compute_ik → solved');
        return { joints, solver: 'moveit' };
      }
      console.error('[IK] MoveIt /compute_ik returned no solution (error_code ≠ 1). Check MoveIt logs.');
      return null;
    } catch (e) {
      console.error('[IK] MoveIt /compute_ik threw an error:', e);
      console.error('[IK] To use IK without a backend, switch to Offline IK mode in Settings → Motion.');
      return null;
    }
  }

  private callMoveItIK(
    x_mm: number, y_mm: number, z_mm: number,
    rx: number, ry: number, rz: number, rw: number,
    currentJoints: number[]
  ): Promise<number[] | null> {
    return new Promise((resolve) => {
      if (!this.ikService) { resolve(null); return; }

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
              position: { x: x_mm / 1000.0, y: y_mm / 1000.0, z: z_mm / 1000.0 },
              orientation: { x: rx, y: ry, z: rz, w: rw }
            }
          },
          timeout: { sec: 5, nanosec: 0 }
        }
      };

      this.ikService!.callService(request, (result: any) => {
        if (result.error_code && result.error_code.val === 1) {
          const positions = result.solution.joint_state.position;
          const idx = result.solution.joint_state.name.indexOf('joint_1');
          if (idx !== -1 && positions.length >= idx + 6) {
            resolve(positions.slice(idx, idx + 6) as number[]);
          } else {
            resolve(positions.slice(0, 6) as number[]);
          }
        } else {
          resolve(null);
        }
      }, () => {
        resolve(null);
      });
    });
  }

  /** Publish a joint trajectory to the ROS bridge */
  public publishTrajectory(joints: number[], durationSec: number = 1.0) {
    if (this.mode === 'offline' || !this.trajectoryTopic) {
      console.log('[Bridge] Offline — trajectory not published to ROS');
      return;
    }

    const trajectoryMsg = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' },
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
    console.log('[Bridge] Published trajectory:', joints.map(j => j.toFixed(3)));
  }

  /** Publish scene objects (obstacles, tables) to ROS for MoveIt collision avoidance */
  public publishSceneObjects(objects: any[]) {
    if (this.mode === 'offline' || !this.ros) return;
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/scene_update',
      messageType: 'std_msgs/String'
    });
    topic.publish({ data: JSON.stringify(objects) });
    console.log('[Bridge] Published scene update to ROS');
  }

  /** Set the tunnel URL and reconnect */
  public setTunnelUrl(url: string) {
    if (typeof localStorage !== 'undefined') {
      localStorage.setItem(TUNNEL_URL_KEY, url);
    }
    // Reconnect with new settings
    if (this.ros) {
      try { this.ros.close(); } catch (_) { /* ignore */ }
    }
    this.autoConnect();
  }

  public getTunnelUrl(): string {
    if (typeof localStorage !== 'undefined') {
      return localStorage.getItem(TUNNEL_URL_KEY) || '';
    }
    return '';
  }

  // ── Hardware Configuration ──

  /** Publish motor configuration for a joint to the ROS bridge */
  public publishMotorConfig(jointIdx: number, config: {
    motor_type: string; pid: { kp: number; ki: number };
    gear_ratio: number; max_current_a: number; signal_type: string;
  }) {
    if (this.mode === 'offline' || !this.ros) {
      console.log(`[Bridge] Offline — motor config for J${jointIdx + 1} stored locally only`);
      return;
    }
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/motor_config',
      messageType: 'std_msgs/String'
    });
    topic.publish({ data: JSON.stringify({ joint_idx: jointIdx, ...config }) });
    console.log(`[Bridge] Published motor config for Joint ${jointIdx + 1}:`, config);
  }

  /** Publish encoder configuration for a joint to the ROS bridge */
  public publishEncoderConfig(jointIdx: number, config: {
    encoder_type: string; counts_per_rev: number;
    gear_ratio: number; zero_offset: number; direction: number;
  }) {
    if (this.mode === 'offline' || !this.ros) {
      console.log(`[Bridge] Offline — encoder config for J${jointIdx + 1} stored locally only`);
      return;
    }
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/encoder_config',
      messageType: 'std_msgs/String'
    });
    topic.publish({ data: JSON.stringify({ joint_idx: jointIdx, ...config }) });
    console.log(`[Bridge] Published encoder config for Joint ${jointIdx + 1}:`, config);
  }

  /** Subscribe to raw encoder data for live diagnostics display */
  public onEncoderRaw(cb: (counts: number[]) => void) {
    if (this.mode === 'offline' || !this.ros) return;
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/encoder/raw',
      messageType: 'std_msgs/Float64MultiArray'
    });
    topic.subscribe((msg: any) => {
      if (msg.data && msg.data.length >= 6) {
        cb(msg.data.slice(0, 6));
      }
    });
  }

  /** Subscribe to live motor telemetry (PWM duty cycle, direction, etc) */
  public onMotorTelemetry(cb: (data: number[]) => void) {
    if (this.mode === 'offline' || !this.ros) return;
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/roboforge/motor_telemetry',
      messageType: 'std_msgs/Float64MultiArray'
    });
    topic.subscribe((msg: any) => {
      if (msg.data) cb(msg.data);
    });
  }

  /** Request Power-On Self-Test from the bridge */
  public async requestPOST(): Promise<any> {
    if (this.mode === 'offline' || !this.ros) {
      return { passed: false, error: 'Not connected to backend' };
    }
    return new Promise((resolve) => {
      const service = new ROSLIB.Service({
        ros: this.ros!,
        name: '/roboforge/health_check',
        serviceType: 'std_srvs/srv/Trigger'
      });
      service.callService({}, (result: any) => {
        resolve(result);
      }, () => {
        resolve({ passed: false, error: 'Service call failed' });
      });
    });
  }

  // ── Event listeners ──

  public onStatus(cb: StatusCallback) {
    this.statusListeners.push(cb);
    cb(this.status, this.mode);
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
    this.statusListeners.forEach(cb => cb(this.status, this.mode));
  }

  /**
   * Publish an IO signal to ROS
   * @param signalName - e.g. "DO_Gripper" → publishes to /io/DO_Gripper
   * @param value - boolean for digital, number for analog
   */
  public publishIO(signalName: string, value: boolean | number) {
    if (this.mode === 'offline' || !this.ros) {
      console.warn(`[IO] Offline — not publishing ${signalName}`);
      return;
    }
    const isDigital = typeof value === 'boolean';
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: `/io/${signalName}`,
      messageType: isDigital ? 'std_msgs/Bool' : 'std_msgs/Float64',
      latch: true,
    });
    topic.publish({ data: value } as any);
    console.log(`[IO] Published /io/${signalName} → ${value}`);
  }
}

export const backendConnector = new BackendConnector();
