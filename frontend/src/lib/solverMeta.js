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
};

export const SOLVER_ORDER = [
  'jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start',
  'protein_ik', 'protein_fast', 'fixed_lambda_ik', 'protein_homotopy',
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
};

export function phaseLabel(phase) {
  return PHASE_LABELS[phase] || phase || '—';
}
