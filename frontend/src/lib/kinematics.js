// Forward kinematics for all supported robots, mirroring the backend's DH-parameter
// math exactly (app/core/kinematics.py) so the 3D arm renders the same
// joint positions the solver actually computed -- this is purely a
// rendering helper; all solving happens server-side.

export const UR5_SPEC = {
  name: 'ur5',
  a: [0, -0.425, -0.39225, 0, 0, 0],
  d: [0.089159, 0, 0, 0.10915, 0.09465, 0.0823],
  alpha: [Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0],
  linkRadius: [0.06, 0.05, 0.045, 0.04, 0.04, 0.035],
};

export const PLANAR3DOF_SPEC = {
  name: 'planar3dof',
  a: [0.4, 0.3, 0.2],
  d: [0, 0, 0],
  alpha: [0, 0, 0],
  linkRadius: [0.03, 0.025, 0.02],
};

export const FRANKA_PANDA_SPEC = {
  name: 'franka_panda',
  a: [0, 0, 0, 0.0825, -0.0825, 0, 0.088],
  d: [0.333, 0, 0.316, 0, 0.384, 0, 0.107],
  alpha: [0, -Math.PI / 2, Math.PI / 2, Math.PI / 2, -Math.PI / 2, Math.PI / 2, Math.PI / 2],
  linkRadius: [0.08, 0.07, 0.07, 0.06, 0.06, 0.05, 0.04],
};

export const ROBOT_SPECS = {
  planar3dof:   PLANAR3DOF_SPEC,
  ur5:          UR5_SPEC,
  franka_panda: FRANKA_PANDA_SPEC,
};

// Reasonable idle poses per robot (all within joint limits)
export const ROBOT_IDLE_Q = {
  planar3dof:   [0, 0, 0],
  ur5:          [0, -0.7, 0.9, -0.4, 0.6, 0],
  franka_panda: [0, -0.3, 0, -2.0, 0, 1.8, 0],  // q4 must be negative per Franka limits
};

function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  // Row-major 4x4, matches the backend's _dh_transform exactly.
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

function matMul(A, B) {
  const result = Array.from({ length: 4 }, () => [0, 0, 0, 0]);
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) sum += A[i][k] * B[k][j];
      result[i][j] = sum;
    }
  }
  return result;
}

const IDENTITY = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

/**
 * Returns an array of (n+1) {position: [x,y,z], matrix} for each joint
 * origin in base frame, given joint angles q (length n). Matches the
 * backend's forward_kinematics_chain exactly.
 */
export function forwardKinematicsChain(spec, q) {
  const n = spec.a.length;
  let T = IDENTITY;
  const chain = [{ position: [0, 0, 0], matrix: IDENTITY }];
  for (let i = 0; i < n; i++) {
    const Ti = dhTransform(q[i], spec.d[i], spec.a[i], spec.alpha[i]);
    T = matMul(T, Ti);
    chain.push({
      position: [T[0][3], T[1][3], T[2][3]],
      matrix: T,
    });
  }
  return chain;
}

/** Extracts a [x,y,z] quaternion-free rotation as a 3x3 for a chain entry. */
export function rotationOf(chainEntry) {
  const m = chainEntry.matrix;
  return [
    [m[0][0], m[0][1], m[0][2]],
    [m[1][0], m[1][1], m[1][2]],
    [m[2][0], m[2][1], m[2][2]],
  ];
}

/** 3x3 rotation matrix -> [x,y,z,w] quaternion (three.js convention). */
export function matrixToQuaternion(R) {
  const m00 = R[0][0], m01 = R[0][1], m02 = R[0][2];
  const m10 = R[1][0], m11 = R[1][1], m12 = R[1][2];
  const m20 = R[2][0], m21 = R[2][1], m22 = R[2][2];
  const trace = m00 + m11 + m22;
  let x, y, z, w;
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1.0);
    w = 0.25 / s;
    x = (m21 - m12) * s;
    y = (m02 - m20) * s;
    z = (m10 - m01) * s;
  } else if (m00 > m11 && m00 > m22) {
    const s = 2.0 * Math.sqrt(1.0 + m00 - m11 - m22);
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = 2.0 * Math.sqrt(1.0 + m11 - m00 - m22);
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    const s = 2.0 * Math.sqrt(1.0 + m22 - m00 - m11);
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  return [x, y, z, w];
}

/** Minimum non-adjacent-link segment distance, mirrors the backend's
 * self_collision_min_distance for client-side collision-glow rendering
 * (visual only -- the backend's value is authoritative for metrics). */
export function selfCollisionMinDistance(spec, chainPositions) {
  const nLinks = spec.a.length;
  let minD = Infinity;
  for (let i = 0; i < nLinks - 1; i++) {
    for (let j = i + 2; j < nLinks; j++) {
      const d = segmentSegmentDistance(
        chainPositions[i], chainPositions[i + 1],
        chainPositions[j], chainPositions[j + 1]
      );
      const adjusted = d - (spec.linkRadius[i] + spec.linkRadius[j]);
      if (adjusted < minD) minD = adjusted;
    }
  }
  return Number.isFinite(minD) ? minD : 1.0;
}

function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function add(a, b, s) { return [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]; }
function norm(a) { return Math.sqrt(dot(a, a)); }

function segmentSegmentDistance(p1, p2, p3, p4) {
  const d1 = sub(p2, p1), d2 = sub(p4, p3), r = sub(p1, p3);
  const a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  const eps = 1e-12;
  let s, t;
  if (a <= eps && e <= eps) return norm(sub(p1, p3));
  if (a <= eps) {
    s = 0; t = Math.min(Math.max(f / e, 0), 1);
  } else {
    const c = dot(d1, r);
    if (e <= eps) {
      t = 0; s = Math.min(Math.max(-c / a, 0), 1);
    } else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      s = denom > eps ? Math.min(Math.max((b * f - c * e) / denom, 0), 1) : 0;
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = Math.min(Math.max(-c / a, 0), 1); }
      else if (t > 1) { t = 1; s = Math.min(Math.max((b - c) / a, 0), 1); }
    }
  }
  const c1 = add(p1, d1, s), c2 = add(p3, d2, t);
  return norm(sub(c1, c2));
}
