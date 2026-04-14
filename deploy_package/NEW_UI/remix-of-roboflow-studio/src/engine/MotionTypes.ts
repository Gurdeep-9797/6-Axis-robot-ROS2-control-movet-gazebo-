// Core data types for the robotics execution engine

export interface Transform {
  position: { x: number; y: number; z: number };
  orientation: { qw: number; qx: number; qy: number; qz: number };
}

export interface JointState {
  angles: number[]; // 6 joint angles in radians
  velocities?: number[];
}

export type MotionType = 'MoveJ' | 'MoveL' | 'MoveC';
export type ZoneType = 'fine' | 'z1' | 'z5' | 'z10' | 'z50' | 'z100';

export const ZONE_RADIUS: Record<ZoneType, number> = {
  fine: 0,
  z1: 1,
  z5: 5,
  z10: 10,
  z50: 50,
  z100: 100,
};

export interface MotionBlock {
  id: string;
  type: MotionType;
  target: Transform;
  speed: number;
  accel: number;
  zone: ZoneType;
  circPoint?: Transform; // for MoveC
}

export interface WaitBlock {
  id: string;
  type: 'Wait';
  duration: number;
}

export interface IOBlock {
  id: string;
  type: 'SetDO' | 'GetDI';
  signal: string;
  value?: boolean;
}

export interface LogicBlock {
  id: string;
  type: 'If' | 'While';
  condition: string;
  children: ProgramBlock[];
  elseChildren?: ProgramBlock[];
}

export interface SyncBlock {
  id: string;
  type: 'SyncRobot' | 'WaitRobot';
  targetRobotId: string;
}

export type ProgramBlock = MotionBlock | WaitBlock | IOBlock | LogicBlock | SyncBlock;

export interface RobotModel {
  id: string;
  name: string;
  dh: DHParameter[];
  jointLimits: { min: number; max: number }[];
  maxSpeed: number[];
  reach: number;
}

export interface DHParameter {
  a: number;   // link length
  alpha: number; // link twist
  d: number;   // link offset
  theta: number; // joint angle (variable)
}

export interface TrajectoryPoint {
  time: number;
  joints: number[];
  position: { x: number; y: number; z: number };
  velocity: number;
}

export interface RobotInstance {
  id: string;
  name: string;
  model: string;
  baseTransform: Transform;
  jointState: JointState;
  program: ProgramBlock[];
  color: string;
}

// ABB IRB 6700 DH parameters (simplified)
export const IRB6700_DH: DHParameter[] = [
  { a: 0.32, alpha: -Math.PI / 2, d: 0.78, theta: 0 },
  { a: 1.075, alpha: 0, d: 0, theta: -Math.PI / 2 },
  { a: 0.2, alpha: -Math.PI / 2, d: 0, theta: 0 },
  { a: 0, alpha: Math.PI / 2, d: 1.142, theta: 0 },
  { a: 0, alpha: -Math.PI / 2, d: 0, theta: 0 },
  { a: 0, alpha: 0, d: 0.2, theta: 0 },
];

export const IRB6700_LIMITS = [
  { min: -170, max: 170 },
  { min: -65, max: 85 },
  { min: -180, max: 70 },
  { min: -300, max: 300 },
  { min: -130, max: 130 },
  { min: -360, max: 360 },
];
