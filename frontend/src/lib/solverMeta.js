export const SOLVERS = {
  jacobian_dls: { name: 'Jacobian (DLS)', short: 'DLS', color: '#5B6B66', family: 'classical' },
  ccd: { name: 'CCD', short: 'CCD', color: '#5B6B66', family: 'classical' },
  fabrik: { name: 'FABRIK', short: 'FABRIK', color: '#5B6B66', family: 'classical' },
  trac_ik_style: { name: 'TRAC-IK style', short: 'TRAC-IK', color: '#E8B95C', family: 'production' },
  multi_start: { name: 'Multi-start', short: 'Multi-start', color: '#E8B95C', family: 'production' },
  protein_ik: { name: 'ProteinIK V1', short: 'ProIK v1', color: '#B30059', family: 'protein' },
  protein_ik_v2: { name: 'ProteinIK V2', short: 'ProIK v2', color: '#FF007F', family: 'protein' },
  protein_ik_v3: { name: 'ProteinIK V3', short: 'ProIK v3', color: '#FF2D95', family: 'protein' },
  protein_ik_v4: { name: 'ProteinIK V4', short: 'ProIK v4', color: '#FF5CAD', family: 'protein' },
};

export const SOLVER_ORDER = [
  'jacobian_dls', 'ccd', 'fabrik', 'trac_ik_style', 'multi_start', 'protein_ik', 'protein_ik_v2', 'protein_ik_v3', 'protein_ik_v4'
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
  funnel_lm_endgame: 'LM endgame (downhill)',
  chaperone_rescue_1: 'chaperone rescue (1 joint)',
  chaperone_rescue_3: 'chaperone rescue (3 joints)',
  chaperone_rescue_5: 'chaperone rescue (5 joints)',
  full_reseed_after_exhausted_rescue: 'full reseed',
  stability_check_passed: 'stability check — passed',
  stability_check_failed: 'stability check — failed',
  dls: 'gradient step',
  ccd_sweep: 'CCD sweep',
  fabrik_pass: 'FABRIK pass',
  dls_attempt: 'gradient attempt',
};

export function phaseLabel(phase) {
  return PHASE_LABELS[phase] || phase || '—';
}
