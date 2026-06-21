/**
 * Kinematics unit tests — vitest
 *
 * Verifies that the JS forward-kinematics implementation matches
 * the backend's DH math numerically. Run with:
 *   npx vitest run
 */
import { describe, it, expect } from 'vitest';
import {
  UR5_SPEC,
  forwardKinematicsChain,
  matrixToQuaternion,
  selfCollisionMinDistance,
} from '../src/lib/kinematics';

const N = UR5_SPEC.a.length; // 6

// ─── helpers ─────────────────────────────────────────────────────────────────

function approxEqual(a, b, tol = 1e-9) {
  return Math.abs(a - b) < tol;
}

function mat4Det(m) {
  // Cofactor expansion on the 3x3 rotation sub-block (last row/col = [0,0,0,1])
  const R = [[m[0][0], m[0][1], m[0][2]],
             [m[1][0], m[1][1], m[1][2]],
             [m[2][0], m[2][1], m[2][2]]];
  return (
    R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0])
  );
}

// ─── FK chain structure ───────────────────────────────────────────────────────

describe('forwardKinematicsChain', () => {
  it('returns n+1 frames for n joints', () => {
    const chain = forwardKinematicsChain(UR5_SPEC, new Array(N).fill(0));
    expect(chain.length).toBe(N + 1);
  });

  it('first frame is at the base origin [0,0,0]', () => {
    const chain = forwardKinematicsChain(UR5_SPEC, new Array(N).fill(0));
    expect(chain[0].position).toEqual([0, 0, 0]);
  });

  it('every rotation sub-matrix has det ≈ +1 (proper rotation)', () => {
    const q = [0.1, -0.5, 0.3, -0.2, 0.7, -0.1];
    const chain = forwardKinematicsChain(UR5_SPEC, q);
    for (const frame of chain) {
      const det = mat4Det(frame.matrix);
      expect(det).toBeCloseTo(1.0, 9);
    }
  });

  it('end-effector position matches last frame position', () => {
    const q = [0.2, -0.4, 0.6, 0.1, -0.3, 0.5];
    const chain = forwardKinematicsChain(UR5_SPEC, q);
    const last = chain[N];
    expect(last.position[0]).toBeCloseTo(last.matrix[0][3], 12);
    expect(last.position[1]).toBeCloseTo(last.matrix[1][3], 12);
    expect(last.position[2]).toBeCloseTo(last.matrix[2][3], 12);
  });

  it('reproducible: same q gives same result twice', () => {
    const q = [0.3, -0.6, 0.9, -0.3, 0.6, -0.9];
    const a = forwardKinematicsChain(UR5_SPEC, q);
    const b = forwardKinematicsChain(UR5_SPEC, q);
    for (let i = 0; i <= N; i++) {
      expect(a[i].position).toEqual(b[i].position);
    }
  });

  it('all-zero joints: base joint is at z=d[0]', () => {
    // With all-zero joint angles, joint 1 should be directly above the base
    // by d[0] = 0.089159 m (the DH shoulder offset).
    const chain = forwardKinematicsChain(UR5_SPEC, new Array(N).fill(0));
    // Position of joint 1 in the chain (index 1)
    const p1 = chain[1].position;
    expect(p1[2]).toBeCloseTo(UR5_SPEC.d[0], 5);
  });
});

// ─── matrixToQuaternion ───────────────────────────────────────────────────────

describe('matrixToQuaternion', () => {
  it('identity matrix → quaternion [0,0,0,1]', () => {
    const R = [[1,0,0],[0,1,0],[0,0,1]];
    const [x, y, z, w] = matrixToQuaternion(R);
    expect(x).toBeCloseTo(0, 12);
    expect(y).toBeCloseTo(0, 12);
    expect(z).toBeCloseTo(0, 12);
    expect(w).toBeCloseTo(1, 12);
  });

  it('90° rotation about Z → x=0, y=0, z≈0.707, w≈0.707', () => {
    const c = Math.cos(Math.PI / 4), s = Math.sin(Math.PI / 4);
    const R = [[c, -s, 0],[s, c, 0],[0, 0, 1]];
    const [x, y, z, w] = matrixToQuaternion(R);
    expect(x).toBeCloseTo(0, 9);
    expect(y).toBeCloseTo(0, 9);
    expect(z).toBeCloseTo(Math.SQRT1_2, 9);
    expect(w).toBeCloseTo(Math.SQRT1_2, 9);
  });

  it('unit norm for arbitrary rotation', () => {
    const q = [0.2, -0.4, 0.6, 0.1, -0.3, 0.5];
    const chain = forwardKinematicsChain(UR5_SPEC, q);
    for (const frame of chain) {
      const R = [[frame.matrix[0][0], frame.matrix[0][1], frame.matrix[0][2]],
                 [frame.matrix[1][0], frame.matrix[1][1], frame.matrix[1][2]],
                 [frame.matrix[2][0], frame.matrix[2][1], frame.matrix[2][2]]];
      const [x, y, z, w] = matrixToQuaternion(R);
      const norm = Math.sqrt(x*x + y*y + z*z + w*w);
      expect(norm).toBeCloseTo(1.0, 9);
    }
  });
});

// ─── selfCollisionMinDistance ─────────────────────────────────────────────────

describe('selfCollisionMinDistance', () => {
  it('returns a finite number for a valid config', () => {
    const chain = forwardKinematicsChain(UR5_SPEC, new Array(N).fill(0));
    const positions = chain.map((f) => f.position);
    const d = selfCollisionMinDistance(UR5_SPEC, positions);
    expect(Number.isFinite(d)).toBe(true);
  });

  it('fully-extended arm has positive clearance', () => {
    // All zeros is roughly "arm stretched out"
    const chain = forwardKinematicsChain(UR5_SPEC, new Array(N).fill(0));
    const positions = chain.map((f) => f.position);
    const d = selfCollisionMinDistance(UR5_SPEC, positions);
    expect(d).toBeGreaterThan(0);
  });
});
