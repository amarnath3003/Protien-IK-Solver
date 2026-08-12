# Research Forks — Rescuing V5 & Raw beyond IK

Two independent, falsifiable research forks were run to test whether the *novel technology* in
**V5 (CCH-IK)** and **V6 (Raw)** has value **outside** the inverse-kinematics task where both were
honest nulls as solvers. Each fork ran real, seeded experiments and was instructed to report a
blunt WIN / PARTIAL / NULL — this project prizes honest negative results.

| Fork | Hypothesis | Verdict | Doc |
|---|---|:---:|---|
| **A — Redundant-robot arena** | The folding physics does real work on a *genuinely* redundant chain (high-DOF planar arm), where Franka's 1 spare DOF couldn't. | **NULL** | [forkA_redundant_robots.md](forkA_redundant_robots.md) |
| **B — Difficulty diagnostics** | V5's conflict-integral and Raw's Σ are useful *label-free* problem-hardness predictors. | **NULL** | [forkB_difficulty_diagnostics.md](forkB_difficulty_diagnostics.md) |

## Bottom line: neither fork produced a win

- **Fork A:** clearance headroom *shrinks* with redundancy (planar chains are space-filling:
  clash-free-solution fraction 22% → 0% as links go 6 → 20), and where headroom exists the physics
  is a statistical wash with random restarts at equal restart budget — and **loses outright at
  equal wall-time** (Raw is 10–100× slower). Raw beats V4 only *intramurally*.
- **Fork B:** every ProteinIK diagnostic is statistical noise vs measured difficulty (V5 ρ≈0.12
  p≈.45; Σ ρ≈0.06 p≈.7). Even the trivial manipulability baseline edges them out. V5 and Σ are
  genuinely orthogonal (ρ=−0.015) — but both fail, so complementarity is moot.

Together these close the **last two escape hatches** for V5/Raw: the "redundant-arm arena" and the
"difficulty-instrument reframe." Combined with the earlier findings (V5's success claim washes at
N=100; Raw ties V4 only via multi-start + clash-free selection, not the fold; Franka self-collision
structurally pinned), the evidence is now consistent and complete.

## What this means & what can be done

See **[what_can_be_done.md](what_can_be_done.md)** for the realistic paths forward. Short version:
the project's genuine, defensible contributions are **V1 (staging beats naive baselines)** and
**V4 (fast, high-success, collision-aware)**. The honest and *publishable* story is the spectrum
itself — **biological depth pays off at the architecture level and stops paying off deeper** — with
V5/Raw documented as rigorous negative results, not buried.
