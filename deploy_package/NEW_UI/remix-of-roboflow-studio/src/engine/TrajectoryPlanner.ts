// Trajectory Planning with trapezoidal velocity profile and zone blending
import { TrajectoryPoint, ZoneType, ZONE_RADIUS, MotionType } from './MotionTypes';

interface WaypointTarget {
  position: { x: number; y: number; z: number };
  motionType: MotionType;
  speed: number;
  accel: number;
  zone: ZoneType;
}

// Trapezoidal velocity profile for a single segment
function trapezoidalProfile(
  distance: number,
  maxSpeed: number,
  accel: number,
  dt: number = 0.01
): { t: number; s: number; v: number }[] {
  if (distance <= 0) return [{ t: 0, s: 0, v: 0 }];

  const tAccel = maxSpeed / accel;
  const dAccel = 0.5 * accel * tAccel * tAccel;

  let profile: { t: number; s: number; v: number }[] = [];

  if (2 * dAccel >= distance) {
    // Triangle profile — can't reach max speed
    const tPeak = Math.sqrt(distance / accel);
    const vPeak = accel * tPeak;
    const totalTime = 2 * tPeak;

    for (let t = 0; t <= totalTime; t += dt) {
      let s: number, v: number;
      if (t <= tPeak) {
        v = accel * t;
        s = 0.5 * accel * t * t;
      } else {
        const td = t - tPeak;
        v = vPeak - accel * td;
        s = distance - 0.5 * accel * (totalTime - t) ** 2;
      }
      profile.push({ t, s: Math.min(s, distance), v: Math.max(v, 0) });
    }
  } else {
    // Full trapezoidal
    const dCruise = distance - 2 * dAccel;
    const tCruise = dCruise / maxSpeed;
    const totalTime = 2 * tAccel + tCruise;

    for (let t = 0; t <= totalTime; t += dt) {
      let s: number, v: number;
      if (t <= tAccel) {
        v = accel * t;
        s = 0.5 * accel * t * t;
      } else if (t <= tAccel + tCruise) {
        v = maxSpeed;
        s = dAccel + maxSpeed * (t - tAccel);
      } else {
        const td = t - tAccel - tCruise;
        v = maxSpeed - accel * td;
        s = dAccel + dCruise + maxSpeed * td - 0.5 * accel * td * td;
      }
      profile.push({ t, s: Math.min(s, distance), v: Math.max(v, 0) });
    }
  }

  return profile;
}

// Interpolate between two 3D points
function lerp3(
  a: { x: number; y: number; z: number },
  b: { x: number; y: number; z: number },
  t: number
) {
  return {
    x: a.x + (b.x - a.x) * t,
    y: a.y + (b.y - a.y) * t,
    z: a.z + (b.z - a.z) * t,
  };
}

function dist3(a: { x: number; y: number; z: number }, b: { x: number; y: number; z: number }) {
  return Math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2);
}

// Generate blended path points between waypoints
export function generateBlendedPath(
  waypoints: WaypointTarget[]
): { x: number; y: number; z: number; motionType: MotionType }[] {
  if (waypoints.length < 2) return waypoints.map(w => ({ ...w.position, motionType: w.motionType }));

  const path: { x: number; y: number; z: number; motionType: MotionType }[] = [];
  const resolution = 40;

  for (let i = 0; i < waypoints.length; i++) {
    const wp = waypoints[i];
    const blendR = ZONE_RADIUS[wp.zone] || 0;

    if (i === 0) {
      // First point — just add approach to blend zone of next segment
      path.push({ ...wp.position, motionType: wp.motionType });
      continue;
    }

    const prev = waypoints[i - 1];
    const segDist = dist3(prev.position, wp.position);

    if (blendR <= 0 || i === waypoints.length - 1) {
      // Fine stop or last waypoint — straight line
      for (let j = 1; j <= resolution; j++) {
        const t = j / resolution;
        const p = lerp3(prev.position, wp.position, t);
        path.push({ ...p, motionType: wp.motionType });
      }
    } else if (i < waypoints.length - 1) {
      const next = waypoints[i + 1];
      const blendFraction = Math.min(blendR / segDist, 0.4);

      // Line segment up to blend start
      for (let j = 1; j <= resolution * (1 - blendFraction); j++) {
        const t = j / (resolution * (1 - blendFraction)) * (1 - blendFraction);
        const p = lerp3(prev.position, wp.position, t);
        path.push({ ...p, motionType: wp.motionType });
      }

      // Quadratic Bézier blend through the waypoint
      const blendStart = lerp3(prev.position, wp.position, 1 - blendFraction);
      const blendEnd = lerp3(wp.position, next.position, blendFraction);
      const blendSteps = Math.max(10, Math.floor(resolution * blendFraction * 2));

      for (let j = 0; j <= blendSteps; j++) {
        const t = j / blendSteps;
        const a = lerp3(blendStart, wp.position, t);
        const b = lerp3(wp.position, blendEnd, t);
        const p = lerp3(a, b, t);
        path.push({ ...p, motionType: wp.motionType });
      }
    }
  }

  return path;
}

// Generate full trajectory with time parameterization
export function generateTrajectory(
  waypoints: WaypointTarget[],
  dt: number = 0.02
): TrajectoryPoint[] {
  const trajectory: TrajectoryPoint[] = [];
  let globalTime = 0;

  for (let i = 1; i < waypoints.length; i++) {
    const prev = waypoints[i - 1];
    const curr = waypoints[i];
    const distance = dist3(prev.position, curr.position);
    const profile = trapezoidalProfile(distance, curr.speed, curr.accel, dt);

    for (const p of profile) {
      const t = distance > 0 ? p.s / distance : 0;
      const pos = lerp3(prev.position, curr.position, t);
      trajectory.push({
        time: globalTime + p.t,
        joints: [],
        position: pos,
        velocity: p.v,
      });
    }

    if (profile.length > 0) {
      globalTime += profile[profile.length - 1].t;
    }
  }

  return trajectory;
}

// Compute circular arc points for MoveC
export function computeCircularArc(
  start: { x: number; y: number; z: number },
  via: { x: number; y: number; z: number },
  end: { x: number; y: number; z: number },
  segments: number = 32
): { x: number; y: number; z: number }[] {
  // Approximate with quadratic Bézier
  const points: { x: number; y: number; z: number }[] = [];
  for (let i = 0; i <= segments; i++) {
    const t = i / segments;
    const a = lerp3(start, via, t);
    const b = lerp3(via, end, t);
    points.push(lerp3(a, b, t));
  }
  return points;
}
