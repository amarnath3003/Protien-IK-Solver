# Fork B — Difficulty Diagnostics

**Question:** Independent of whether the *solvers* win, did ProteinIK produce two genuinely useful
**label-free problem-difficulty diagnostics**?
- **V5 conflict / difficulty score** — instance hardness from optimization-dynamics gradient
  conflict (cosine between task and constraint gradients, integrated over the solve).
- **Raw Σ** (Bryngelson–Wolynes funnel-vs-glass ratio) — energy-landscape topology (funnel = easy
  vs glass = hard), measured **before** solving.

**Method.** Per-target **ground-truth difficulty** on UR5 across open_space / near_singular /
cluttered: `y_fail` = fraction of K random restarts that fail to reach the target, and `y_iter` =
mean iterations-to-converge. Predict `y` from each diagnostic; score with **Spearman rank
correlation**. Also correlate predictors with each other (complementary vs redundant?) and against
a cheap **manipulability** baseline. Seeded.

---

## Result — every ProteinIK diagnostic is statistical noise vs measured difficulty

Spearman ρ (UR5, n=42; open + near_singular + partial cluttered):

| Predictor | ρ vs y_fail | ρ vs y_iter |
|---|---|---|
| V5 difficulty_score | +0.121 (p=.45) | +0.156 (p=.32) |
| V5 conflict_index   | +0.054 (p=.73) | +0.025 (p=.88) |
| Raw Σ               | +0.064 (p=.69) | −0.050 (p=.76) |
| **manipulability baseline** | +0.114 (p=.47) | **−0.271 (p=.083)** |

> _Final data: UR5, n=42 (20 open + 20 near_singular + 2 cluttered, under-sampled); Franka never
> completed (n=2, excluded). p-values of 0.3–0.9 and 95% bootstrap CIs that all straddle 0 mean no
> ProteinIK predictor is distinguishable from zero. The point estimates aren't merely
> non-significant — they're near zero (|ρ|≤0.16)._

**Within-scenario** (removes the scenario-label confound — correlate only *inside* open / near_sing):
V5 difficulty mean ρ = **+0.005**, Raw Σ mean ρ = **+0.10**, manipulability mean ρ = +0.057. Still
noise — the diagnostics don't separate hard from easy instances *within* a regime either.

Ground-truth labels have real spread (`y_fail` 0.17→0.78, `y_iter` 10→200), so difficulty genuinely
varies — the predictors simply don't track it. `y_iter` is the more reliable label (K=40 gives
binomial noise on `y_fail`), and the diagnostics are flat against it too.

**Predictor ⟂ predictor:**
- V5 difficulty vs Raw Σ = **−0.015** → **essentially orthogonal** (they genuinely measure
  different things — dynamics vs topology)…
- …**but both fail to predict difficulty, so the complementarity is moot.** Two independent noise
  sources are still noise.
- V5 difficulty vs V5 conflict = **+0.651** → V5's own two diagnostics are largely redundant with
  each other.
- Σ vs manipulability = +0.298.

---

## Verdict: **NULL** (both diagnostics)

- **V5 conflict / difficulty:** NULL — ρ ≈ +0.05…+0.16, p ≈ 0.3–0.9. Does not predict measured
  instance hardness. (Consistent with the earlier finding that V5's *success* claim was also a
  wash at N=100 — the conflict signal simply doesn't track real difficulty.)
- **Raw Σ:** NULL — ρ ≈ ±0.06, p ≈ 0.7. The funnel/glass ratio does not predict solve difficulty on
  UR5. (Matches the mixed/weak Phase-4 signal −0.24/+0.16/−0.12; on a clean per-target ground truth
  it's indistinguishable from zero.)
- Even the **cheap manipulability baseline** shows only a weak trend (ρ=−0.27, p=.083) — and it
  still edges out both ProteinIK diagnostics. There is no case where a ProteinIK diagnostic is the
  best predictor.

This **closes the "difficulty-meter reframe"** — the idea that V5/Raw's surviving value was as
label-free hardness instruments. On measured ground truth, they aren't.

---

## What would need to be true instead

- A domain where the diagnostics **do** track a measurable outcome — but the natural one (IK solve
  difficulty) just failed, and it was the most favorable case (their home turf).
- Σ would need testing on landscapes with a *known* funnel/glass structure (spin-glass / protein
  toy models) where its physics assumptions hold — i.e. **not** IK. That is a different project, and
  Σ there would be competing with established foldability metrics.
- The orthogonality (V5 ⟂ Σ) is real but only becomes useful if at least one of them predicts
  *something*. Neither does here.

_Scripts: `scratchpad/forkB/` (seeded). Ground truth = restart-failure fraction + iteration count._
