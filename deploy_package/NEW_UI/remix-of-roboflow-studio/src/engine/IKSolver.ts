/**
 * IKSolver.ts — OFFLINE IK ONLY
 *
 * This module is the LOCAL JavaScript fallback solver. It is used ONLY when
 * the user has explicitly selected an offline IK mode in Settings.
 *
 * PRIMARY IK PATH: MoveIt 2 via /compute_ik service (rosbridge WebSocket).
 * OFFLINE PATH: This file — activated only via Settings toggle.
 *
 * NEVER call solveIK() directly from execution code. Always route through
 * BackendConnector.computeIK() which enforces the mode policy.
 */

import { Transform, DHParameter, IRB6700_DH, IRB6700_LIMITS } from './MotionTypes';

/**
 * IKMode — controls which solver is used.
 * 'moveit'              → MoveIt 2 via /compute_ik ROS service (DEFAULT — requires backend)
 * 'offline-analytical'  → Local JS analytical solver (settings toggle only)
 * 'offline-numerical'   → Local JS numerical DLS solver (settings toggle only)
 */
export type IKMode = 'moveit' | 'offline-analytical' | 'offline-numerical';

// ─── Proper DH Forward Kinematics ───────────────────────────────────────────
// Uses standard DH convention: T_i = Rz(θ) · Tz(d) · Tx(a) · Rx(α)

function dhMatrix(a: number, alpha: number, d: number, theta: number): number[] {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  // 4x4 homogeneous matrix in row-major (flat array of 16)
  return [
    ct,     -st * ca,  st * sa,  a * ct,
    st,      ct * ca, -ct * sa,  a * st,
    0,       sa,       ca,       d,
    0,       0,        0,        1
  ];
}

function matMul4(A: number[], B: number[]): number[] {
  const R = new Array(16).fill(0);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      for (let k = 0; k < 4; k++) {
        R[i * 4 + j] += A[i * 4 + k] * B[k * 4 + j];
      }
    }
  }
  return R;
}

export function forwardKinematics(joints: number[], dh: DHParameter[] = IRB6700_DH): Transform {
  // Identity matrix
  let T: number[] = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  ];

  for (let i = 0; i < Math.min(joints.length, dh.length); i++) {
    const theta = joints[i] + dh[i].theta;
    const Ti = dhMatrix(dh[i].a, dh[i].alpha, dh[i].d, theta);
    T = matMul4(T, Ti);
  }

  // Extract position in meters, then convert to mm for the workspace
  const x_mm = T[3]  * 1000; // T[0][3]
  const y_mm = T[7]  * 1000; // T[1][3]
  const z_mm = T[11] * 1000; // T[2][3]

  return {
    position: { x: x_mm, y: y_mm, z: z_mm },
    orientation: { qw: 1, qx: 0, qy: 0, qz: 0 },
  };
}

// ─── Numerical IK using Jacobian pseudo-inverse (DLS) ────────────────────────

function solveIKNumerical(
  target: Transform,
  initialJoints: number[] = [0, 0, 0, 0, 0, 0],
  maxIterations = 200,
  tolerance = 2.0, // mm
  regularization = 0.05 // Penalty for joint distance from initialJoints
): { success: boolean; joints: number[]; iterations: number; error: number } {
  const joints = [...initialJoints];
  const dt = 0.0001; 
  const learningRate = 0.5;
  const lambda = 0.1; 

  for (let iter = 0; iter < maxIterations; iter++) {
    const current = forwardKinematics(joints);
    const dx = target.position.x - current.position.x;
    const dy = target.position.y - current.position.y;
    const dz = target.position.z - current.position.z;
    const error = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (error < tolerance) {
      return { success: true, joints, iterations: iter, error };
    }

    // Compute Jacobian numerically (3x6)
    const J: number[][] = [[], [], []];
    for (let j = 0; j < 6; j++) {
      const perturbed = [...joints];
      perturbed[j] += dt;
      const fk = forwardKinematics(perturbed);
      J[0][j] = (fk.position.x - current.position.x) / dt;
      J[1][j] = (fk.position.y - current.position.y) / dt;
      J[2][j] = (fk.position.z - current.position.z) / dt;
    }

    // Damped Least Squares (DLS) step with Joint Regularization
    const errorVec = [dx, dy, dz];
    for (let j = 0; j < 6; j++) {
      let dtheta = 0;
      for (let k = 0; k < 3; k++) {
        dtheta += J[k][j] * errorVec[k];
      }
      
      // Add regularization term: steer towards initialJoints to maintain configuration
      const jointDist = joints[j] - initialJoints[j];
      dtheta -= regularization * jointDist;

      const norm = J[0][j] ** 2 + J[1][j] ** 2 + J[2][j] ** 2 + lambda * lambda;
      joints[j] += (dtheta / norm) * learningRate;

      // Clamp to limits
      const lim = IRB6700_LIMITS[j];
      const rad = (deg: number) => (deg * Math.PI) / 180;
      joints[j] = Math.max(rad(lim.min), Math.min(rad(lim.max), joints[j]));
    }
  }

  const finalFK = forwardKinematics(joints);
  const finalError = Math.sqrt(
    (target.position.x - finalFK.position.x) ** 2 +
    (target.position.y - finalFK.position.y) ** 2 +
    (target.position.z - finalFK.position.z) ** 2
  );

  return { success: finalError < 20, joints, iterations: maxIterations, error: finalError };
}

// Multi-seed numerical IK — tries multiple starting configurations
function solveIKMultiSeed(
  target: Transform,
  currentJoints?: number[]
): { success: boolean; joints: number[]; iterations: number; error: number } {
  const primarySeed = currentJoints || [0, 0, 0, 0, 0, 0];
  
  // Try previous pose first — strongly weighted to stay in same configuration
  const primaryResult = solveIKNumerical(target, primarySeed, 500, 1.0, 0.1);
  if (primaryResult.success) return primaryResult;

  const seeds: number[][] = [
    [0, 0, 0, 0, 0, 0],
    [0, -0.5, 0.3, 0, -0.2, 0],
    [0, -1.0, 0.5, 0, -0.3, 0],
    [0.5, -0.3, 0.2, 0, 0, 0],
    [-0.5, -0.3, 0.2, 0, 0, 0],
    [0, 0.3, -0.2, 0, 0, 0],
    [0, -0.7, 0.8, 0, -0.5, 0],
    [0, -1.2, 0.8, 0, 0, 0],       
  ];

  let bestResult = primaryResult;

  for (const seed of seeds) {
    const result = solveIKNumerical(target, seed, 400, 2.0, 0.02);
    if (result.success) {
      // Compare joint distance to primarySeed — if too far, we might want to warn the user
      // about a configuration flip.
      return result;
    }
    if (result.error < bestResult.error) bestResult = result;
  }

  return bestResult;
}

// ─── Analytical IK for 6-axis industrial arm ─────────────────────────────────

export function solveIKAnalytical(
  target: Transform,
): { success: boolean; joints: number[]; error: number } {
  const { x, y, z } = target.position;

  // Reach validation — reject unreachable targets early
  const reachMm = Math.sqrt(x * x + y * y + z * z);
  const MIN_REACH_MM = 350;
  const MAX_REACH_MM = 2550;
  if (reachMm < MIN_REACH_MM || reachMm > MAX_REACH_MM) {
    return { success: false, joints: [0, 0, 0, 0, 0, 0], error: reachMm };
  }

  // DH link lengths (meters) from the IRB 6700 DH params
  const d1 = 0.78;   // base height
  const a2 = 1.075;  // upper arm
  const d4 = 1.142;  // forearm
  const a3 = 0.2;    // elbow offset

  const scale = 0.001; // mm to m
  const px = x * scale, py = y * scale, pz = z * scale;

  // Joint 1: base rotation
  const j1 = Math.atan2(py, px);

  // Wrist center position (simplified — ignore tool length for now)
  const r = Math.sqrt(px * px + py * py);
  const s = pz - d1;

  // Distance from shoulder to wrist center
  const L3 = Math.sqrt(a3 * a3 + d4 * d4); // effective forearm length
  const D = Math.sqrt(r * r + s * s);

  // Check triangle inequality
  if (D > a2 + L3 || D < Math.abs(a2 - L3)) {
    return { success: false, joints: [0, 0, 0, 0, 0, 0], error: D * 1000 };
  }

  // Joint 3: elbow angle using law of cosines
  const cosJ3 = (D * D - a2 * a2 - L3 * L3) / (2 * a2 * L3);
  const clampedCos = Math.max(-1, Math.min(1, cosJ3));
  const j3raw = Math.acos(clampedCos);

  // Account for elbow offset angle
  const phi3 = Math.atan2(d4, a3);
  const j3 = -(Math.PI - j3raw) + phi3;

  // Joint 2: shoulder angle
  const beta = Math.atan2(s, r);
  const psi = Math.atan2(L3 * Math.sin(j3raw), a2 + L3 * Math.cos(j3raw));
  const j2 = beta - psi;

  // Wrist joints (simplified — tool pointing down)
  const j4 = 0;
  const j5 = -(j2 + j3); // keep TCP roughly vertical
  const j6 = 0;

  const joints = [j1, j2, j3, j4, j5, j6];

  // Verify with FK
  const fk = forwardKinematics(joints);
  const error = Math.sqrt(
    (target.position.x - fk.position.x) ** 2 +
    (target.position.y - fk.position.y) ** 2 +
    (target.position.z - fk.position.z) ** 2
  );

  return { success: error < 150, joints, error };
}

/**
 * solveIK — LOCAL offline solver entry point.
 *
 * This function MUST only be called when ikMode is 'offline-analytical' or
 * 'offline-numerical'. BackendConnector.computeIK() enforces this policy.
 * If mode is 'moveit', this function should NEVER be called.
 */
export function solveIK(
  target: Transform,
  mode: IKMode = 'offline-numerical',
  currentJoints?: number[]
) {
  if (mode === 'moveit') {
    // Programming error — this should be caught by BackendConnector. Log and use numerical as last resort.
    console.error('[IKSolver] solveIK() called with mode=moveit — this is a bug. IK should go through BackendConnector → MoveIt.');
    return { success: false, joints: [0, 0, 0, 0, 0, 0], error: Infinity };
  }

  if (mode === 'offline-analytical') {
    const analyticalResult = solveIKAnalytical(target);
    if (analyticalResult.success) return analyticalResult;
    // Analytical failed — fall through to numerical using analytical joints as seed
    return solveIKMultiSeed(target, currentJoints || analyticalResult.joints);
  }

  // offline-numerical: multi-seed DLS approach
  const numericalResult = solveIKMultiSeed(target, currentJoints);
  if (numericalResult.success) return numericalResult;
  // Last resort: analytical seed → numerical refinement
  const analyticalSeed = solveIKAnalytical(target);
  if (analyticalSeed.joints.some(j => j !== 0)) {
    return solveIKNumerical(target, analyticalSeed.joints, 500, 5.0);
  }
  return numericalResult;
}

export function jointsToDegrees(joints: number[]): number[] {
  return joints.map(j => (j * 180) / Math.PI);
}

export function degreesToJoints(degrees: number[]): number[] {
  return degrees.map(d => (d * Math.PI) / 180);
}
