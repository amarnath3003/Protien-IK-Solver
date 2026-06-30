export const ROBOTS = {
  planar3dof:   { name: 'Planar 3-DOF', dof: 3, description: '3-link planar RRR arm with closed-form analytical IK.' },
  ur5:          { name: 'UR5 (6-DOF)',   dof: 6, description: 'Universal Robots UR5 — standard 6-DOF benchmark arm.' },
  franka_panda: { name: 'Franka Panda (7-DOF)', dof: 7, description: 'Franka Emika Panda — redundant 7-DOF; null-space enables energy minimization.' },
};

export const ROBOT_ORDER = ['planar3dof', 'ur5', 'franka_panda'];

// Explicit allowlist per robot. analytical_planar3dof is only valid on planar3dof.
export const ROBOT_SOLVER_COMPAT = {
  planar3dof:   ['jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start',
                 'protein_ik', 'protein_fast', 'fixed_lambda_ik', 'protein_homotopy',
                 'protein_raw', 'analytical_planar3dof'],
  ur5:          ['jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start',
                 'protein_ik', 'protein_fast', 'fixed_lambda_ik', 'protein_homotopy',
                 'protein_raw'],
  franka_panda: ['jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start',
                 'protein_ik', 'protein_fast', 'fixed_lambda_ik', 'protein_homotopy',
                 'protein_raw'],
};

export const SOLVERS = {
  jacobian_dls: { name: 'Jacobian (DLS)', short: 'DLS', color: '#5B6B66', family: 'classical' },
  ccd: { name: 'CCD', short: 'CCD', color: '#5B6B66', family: 'classical' },
  fabrik: { name: 'FABRIK', short: 'FABRIK', color: '#5B6B66', family: 'classical' },
  trac_ik_style: { name: 'TRAC-IK style', short: 'TRAC-IK', color: '#E8B95C', family: 'production' },
  multi_start: { name: 'Multi-start', short: 'Multi-start', color: '#E8B95C', family: 'production' },
  protein_ik: { name: 'ProteinIK (V1)', short: 'ProIK v1', color: '#FF007F', family: 'protein' },
  fixed_lambda_ik: {
    name: 'Fixed-λ Homotopy (Baseline)',
    short: 'Fixed-λ',
    color: '#8B9E9A',
    family: 'baseline',
  },
  protein_homotopy: {
    name: 'ProteinIK Homotopy (CCH-IK)',
    short: 'CCH-IK',
    color: '#00D4AA',          // distinct teal — V5 research solver
    family: 'protein',
  },
  protein_fast: {
    name: 'ProteinIK Fast (V4)',
    short: 'ProIK Fast',
    color: '#9D00FF',          // purple for speed/fast variants
    family: 'protein',
  },
  protein_raw: {
    name: 'ProteinIK Raw Biology (V6)',
    short: 'ProIK Raw',
    color: '#D35400',          // burnt orange — raw biophysical folding
    family: 'protein',
  },
  analytical_planar3dof: {
    name: 'Analytical IK (Planar 3-DOF, exact)',
    short: 'Analytical',
    color: '#FFD700',          // gold — exact closed-form, ground truth
    family: 'classical',
  },
};

// Default order shown for all robots (analytical_planar3dof added per-robot via ROBOT_SOLVER_COMPAT)
export const SOLVER_ORDER = [
  'jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start',
  'protein_ik', 'protein_fast', 'fixed_lambda_ik', 'protein_homotopy', 'protein_raw',
];

export const SCENARIOS = {
  open_space: { name: 'Open space', description: 'Uniform random targets, no obstacle bias.' },
  near_singular: { name: 'Near-singular', description: 'Targets biased toward low manipulability.' },
  cluttered: { name: 'Cluttered', description: 'Targets biased toward tight, near-self-collision configurations.' },
};

export const PHASE_LABELS = {
  local_blind_relax: 'local relax (target-blind)',
  coarse_collapse: 'coarse collapse',
  funnel_narrowing: 'funnel narrowing',
  chaperone_rescue: 'chaperone rescue',
  iam_unfold_1: 'IAM rescue (1 joint)',
  iam_unfold_3: 'IAM rescue (3 joints)',
  iam_unfold_5: 'IAM rescue (5 joints)',
  iam_full_unfold: 'IAM full unfold',
  full_reseed_after_exhausted_rescue: 'full reseed',
  stability_check_passed: 'stability check — passed',
  stability_check_failed: 'stability check — failed',
  dls: 'gradient step',
  ccd_sweep: 'CCD sweep',
  fabrik_pass: 'FABRIK pass',
  dls_attempt: 'gradient attempt',
  // CCH-IK phases
  cch_lambda_advance: 'λ advancing — low conflict',
  cch_lambda_hold:    'λ held — conflict detected',
  // Raw Biology (V6) folding phases
  raw_unfolded:       'unfolded (hot Langevin)',
  raw_collapse:       'hydrophobic collapse',
  raw_consolidate:    'tertiary consolidation',
  raw_native_settle:  'native-state settling (T→0)',
  raw_native_stable:  'native state — stable',
};

export function phaseLabel(phase) {
  return PHASE_LABELS[phase] || phase || '—';
}
