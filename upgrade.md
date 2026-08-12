# IMPORTANT — DEFERRED UPGRADE PLAN: DO NOT USE YET

> **DO NOT CONSIDER, APPLY, CITE, OR USE THIS FILE WHILE COMPLETING THE INITIAL DRAFT.**  
> This document is a later-stage upgrade plan. It exists so the project can be strengthened after the initial draft is complete without distracting or biasing the current drafting process. Any AI or contributor reading this repository must ignore this file until the project owner explicitly states that the initial draft is complete and explicitly asks to begin the `upgrade.md` work.

---

# ProteinIK research-paper upgrade report

## 1. Purpose

This document is the complete post-draft plan for turning ProteinIK from a promising, well-engineered research prototype into a rigorous and defensible research contribution.

The present project has real strengths: working code, traceable benchmarks, genuine production baselines, simulator cross-checks, negative-result history, native implementations, and an interesting design intuition. The main weaknesses are not that the work is fake or worthless. They are that the current evidence does not yet isolate the claimed mechanism, several paper statements are contradicted by the designated result files, the collision comparison is not matched against collision-aware baselines, and some statistical and reproducibility requirements are missing.

The upgrade should therefore pursue four goals simultaneously:

1. **Make every quantitative sentence exactly true.**
2. **Separate the useful algorithm from the biological interpretation and test both.**
3. **Compare against baselines that attack the same constrained IK problem.**
4. **Produce artifacts from which an independent reviewer can regenerate every table and figure.**

The best possible final paper is not the one with the most dramatic claims. It is the one whose strongest claims survive matched baselines, component ablations, repeated trials, independent scoring, and hostile review.

---

## 2. Current authoritative artifacts

When this upgrade begins, establish one explicit source-of-truth block in the paper and repository. At the time this report was written, the owner identified these as authoritative:

- Latest draft: `paper/academic.md`
- Success and latency: `backend/results/master_full(cpp).md` and its CSV/manifest
- Collision: `backend/results/master_10seed_fast(cpp).csv` and its manifest
- Solver implementation: `backend/cpp/pik_v4.hpp`, with adapters under `backend/native_bench/`

Older files should remain available as development history, but must not silently feed figures, tables, or prose. A future build script should fail if a paper artifact points to a superseded result file.

Before any new experiment, create a paper release configuration containing:

- Git commit hash;
- dirty-worktree status;
- compiler and flags;
- dependency lock file;
- CPU model, core count, RAM, operating system, and WSL/native status;
- exact solver list;
- exact robot models and URDF checksums;
- exact scenario configuration;
- random seeds;
- trial count;
- timeout and iteration budgets;
- collision-pair allow/deny list;
- result-schema version.

No paper-grade result should be accepted unless it points back to this configuration.

---

## 3. Recommended scientific positioning

### 3.1 Strongest defensible central claim

The safest and potentially strongest positioning is:

> ProteinIK is a collision-aware inverse-kinematics cascade inspired by protein-folding concepts. It combines a cheap multi-seed local phase with conditional escalation to collision-aware stochastic refinement and scoped rescue. The work studies when this cascade improves success, latency, and clean-solve rate over matched IK alternatives.

This framing keeps the biological inspiration but makes the technical object clear. It does not require proving that robot IK and protein folding are mathematically identical.

### 3.2 Claims that should be softened immediately

Replace:

- “structural isomorphism” with “structural analogy” or “shared constrained-chain representation”;
- “the method wins because the problem becomes folding” with “the observed trend is consistent with the hypothesis that collision-aware chain search becomes more valuable as chain length increases”;
- “proteins use this exact sequence” with a carefully sourced statement about the folding concepts that inspired each computational choice;
- “real-time capable” with “sub-millisecond mean solver-core latency on the reported benchmark platform,” unless hard real-time deployment is actually tested;
- “single-shot” with “single solver invocation,” whenever that invocation contains multiple internal replicas or candidate selection;
- “independent physical validation” with “independent software-engine cross-validation on a shared URDF,” unless hardware experiments are added.

### 3.3 A useful two-layer contribution

The final paper can distinguish:

1. **Technical contribution:** a conditional collision-aware IK cascade with scoped escalation and candidate selection.
2. **Design contribution:** a protein-folding-inspired vocabulary and process decomposition that generated testable algorithmic choices.

The technical contribution must stand even if reviewers reject the biological analogy. The biological contribution becomes credible only if its supposedly distinctive components survive matched ablations.

---

## 4. Stop-the-line corrections

These are mandatory factual corrections before submission, independent of any new experiment.

### 4.1 Correct the “cleanest solver” statement

The designated collision CSV contains `protein_raw`, which has lower UR5 PyBullet collision rates than `protein_fast` in all three scenarios while maintaining high success. Therefore, the current claim that KineticFold is the cleanest solver “of any solver in the study” is false.

Choose exactly one defensible resolution:

- Include Protein Raw/LangevinFold in the main comparison and identify it as the cleanest but slower method; or
- formally define a “fast practical solver field,” state its inclusion criteria before presenting results, and say KineticFold is cleanest only within that predefined field; or
- remove Raw from the paper-grade benchmark entirely and explain that it is future work, regenerating the authoritative artifact rather than merely filtering it out of figures.

Do not keep Raw in the authoritative CSV while using hand-selected plotting lists to support “any solver” language.

### 4.2 Correct the collision denominator explanation

The benchmark currently counts collisions across all returned final configurations, including failures. It does not remove failed hard targets from the collision denominator. Delete any sentence saying failed targets are removed from a solver’s denominator.

Report three distinct outcomes:

- `success_rate = P(success)`;
- `collision_given_success = P(collision | success)`;
- `clean_success_rate = P(success AND collision_free)`.

The clean-success rate should be computed per trial as a joint event. Do not estimate it by multiplying aggregate success and collision percentages unless success is exactly 100% and that simplification is explicitly noted.

### 4.3 Correct warm-up documentation

The paper says warm-up targets come from a separate generator, but the benchmark reuses the first benchmark targets. Either change the code to generate disjoint warm-up targets or change the paper. The preferred fix is separate warm-up targets so timed trials have never been solved previously in the process.

### 4.4 Define the solver field once

The paper and figures currently use different implicit solver sets. Create named fields such as:

- `core_field`: StagedFold, KineticFold, TRAC-IK, Multi-start, Jacobian-DLS, CCD, FABRIK;
- `collision_aware_field`: KineticFold plus all matched collision-aware alternatives;
- `extended_field`: core field plus Raw/LangevinFold and internal variants;
- `diagnostic_variants`: calibration and O2 variants, not eligible for headline ranking unless separately justified.

Every table and figure caption must name its field. Avoid “all solvers,” “whole field,” or “any solver” unless literally every eligible solver appears.

### 4.5 Correct latency-superlative language

Recompute rankings separately for mean, median, p95, p99, and maximum. Do not say “smallest tail” if another high-success method has a lower p99. Define whether low-success methods are excluded from latency ranking and why.

### 4.6 Correct the 16-DOF wording

At 16 DOF, the present evidence is one clean KineticFold solution and zero TRAC-IK solutions out of 120. Report counts as well as percentages. Do not call the ratio 2–4× when the baseline is zero. Use wording such as:

> In this sample KineticFold returned 1/120 clean solutions and TRAC-IK returned 0/120; the sample is too small to infer a reliable advantage at this endpoint.

---

## 5. Experimental redesign

## 5.1 Define the primary questions before running anything

The upgraded paper should answer a small set of preregistered questions:

1. Does KineticFold improve pose success over strong production IK solvers at equal or lower measured compute cost?
2. Does it improve clean-success rate over collision-aware baselines, not merely pose-only baselines?
3. Which component causes each improvement: gating, annealing, scoped rescue, collision energy, adaptive budget, or clearance selection?
4. Does the benefit generalize across robot geometry, target distribution, initial state, and collision model?
5. Does the chain-length trend remain after comparing against budget-matched collision-aware selection methods?
6. Is the biological decomposition predictive, or merely a useful naming scheme?

Write these questions into an experiment specification before looking at the new results. For each question, define the primary metric and pass/fail criterion.

## 5.2 Add matched non-biological baselines

This is the single most important experimental upgrade.

### Baseline A: LM cascade

Run the same Phase-A adaptive LM replicas as KineticFold, with the same seeds and budget, but on failure continue with more LM/global restarts instead of folding-inspired Phase B.

Purpose: determine whether conditional multi-start alone explains success and speed.

### Baseline B: annealed IK without chaperone logic

Use the same target, limit, smoothness, and collision energy and the same Metropolis schedule, but use ordinary global random restarts instead of scoped rescue.

Purpose: isolate scoped rescue.

### Baseline C: fixed two-stage cascade

Always run Phase A followed by Phase B, regardless of the frustration gate.

Purpose: isolate conditional gating and verify the latency claim.

### Baseline D: clearance-selecting Multi-start

Run TRAC-IK or LM repeatedly under one of two fair budgets:

- the same number of candidate solves as KineticFold; and
- the same wall-clock budget as KineticFold.

Select the successful candidate with maximum proxy clearance, exactly as KineticFold does internally.

Purpose: determine how much of the collision result comes from candidate selection rather than the search dynamics.

### Baseline E: collision-penalized local optimizer

Use a conventional optimizer such as SQP, SLSQP, L-BFGS-B, or a trust-region least-squares method on the same pose, joint-limit, and collision objective.

Purpose: compare the proposed cascade against a standard constrained optimization formulation using identical energy information.

### Baseline F: established collision-aware IK

Where technically feasible, evaluate at least one established collision-aware system:

- BioIK with collision objectives;
- CollisionIK;
- DawnIK;
- MoveIt/Drake constrained IK with self-collision constraints;
- another peer-reviewed, open implementation appropriate to single-pose IK.

If direct integration is impossible, document the exact incompatibility and include the strongest feasible matched substitute. Do not simply compare collision-aware ProteinIK against pose-only TRAC-IK and call that the complete field.

## 5.3 Full component ablation

Implement a configuration-driven ablation matrix around KineticFold. Every row must use identical targets and preferably common random numbers.

| Component | On condition | Off condition | Primary question |
|---|---|---|---|
| Frustration gate | Phase B conditional | Phase B always runs | Does the gate reduce latency without reducing quality? |
| Collision term | Current weight | Weight zero | Is cleanliness coming from explicit collision information? |
| Metropolis acceptance | Thermal uphill moves | Greedy only | Does annealing escape useful traps? |
| Scoped rescue | Local-to-global ladder | Immediate global restart | Does scope preserve useful partial solutions? |
| Clearance selection | Best clearance | First converged | How much does internal selection contribute? |
| Phase-A replicas | Six | One, two, and budget-matched alternatives | Is success simply multi-start? |
| Target-blind stage | Enabled in a compatible variant | Disabled | Does the distinctive folding stage help? |
| Difficulty scaling | Adaptive | Fixed budget | Is contact-order-inspired scaling useful? |
| Stability gate | Enabled | Disabled | Does it reject meaningfully unstable configurations? |
| Smoothness/neutral terms | Current | Standard regularizer controls | Are biology-named terms different from ordinary posture regularization? |

Use a factorial or fractional-factorial design where possible. One-factor-at-a-time experiments can miss interactions, particularly between gating, replica count, and clearance selection.

The ablation must report success, clean success, collision conditional on success, time distribution, number of candidate evaluations, FK/Jacobian calls, and energy evaluations.

## 5.4 Generalization split

The UR5 was the primary tuning arm. That creates a risk that the observed behavior is a tuned UR5 heuristic.

Use an explicit development/test split:

- **Development robots/scenarios:** permitted for tuning.
- **Held-out robots/scenarios:** frozen until final evaluation.

Possible held-out arms should vary geometry, redundancy, joint-limit asymmetry, and scale. Examples include an additional 6-DOF industrial arm, another 7-DOF redundant arm, and at least one hyper-redundant synthetic chain whose parameters were not used in tuning.

Freeze all ProteinIK weights before running the held-out set. If robot-specific radii or scenario thresholds are required, define a mechanical derivation procedure rather than hand tuning them from benchmark outcomes.

## 5.5 Better scenario design

Keep the current reachable-by-construction targets, but add scenarios that disentangle different difficulties:

1. Uniform reachable poses.
2. Near singular targets.
3. Self-collision-prone target poses.
4. Difficult initial configurations with ordinary targets.
5. Large Cartesian displacement from the initial pose.
6. Large orientation displacement.
7. Near joint-limit targets.
8. Multiple IK branches with both clean and colliding solutions.
9. Targets for which no clean solution exists under the model.
10. Workspace obstacles, if obstacle claims are added.

For collision-focused tests, classify target poses by whether a clean solution is known to exist. A target generated from a colliding configuration may still have a clean alternative, but this must be measured rather than assumed. Use a high-budget oracle or exhaustive/large multi-start search to estimate feasibility.

## 5.6 Trajectory-level evaluation

Single-pose IK is not sufficient to establish control usefulness. Add continuous target sequences and report:

- pose tracking error;
- self-collision events;
- joint velocity, acceleration, and jerk;
- configuration discontinuities;
- branch switching;
- solve deadline misses;
- warm-start behavior from the previous solution;
- recovery after a temporarily infeasible target.

This experiment may reveal that independently selecting maximum-clearance candidates causes discontinuous joint motion. If so, add a displacement/smoothness selection term or position the solver for planning rather than servo control.

## 5.7 Real-robot or hardware-in-the-loop validation

The ideal final paper includes at least one physical arm. Minimum useful hardware validation:

- command a set of safe target poses;
- validate actual end-effector error through calibrated sensing;
- confirm joint-limit compliance;
- demonstrate collision-free motion under conservative safety margins;
- measure end-to-end latency, not only solver-core latency;
- record failure cases and controller safeguards.

If hardware is unavailable, use hardware-in-the-loop or a full planning/control stack and explicitly retain “simulation-only” as a limitation.

---

## 6. Statistical analysis plan

## 6.1 Preserve trial-level results

The current aggregate CSVs are insufficient for the best statistical analysis. Write one row per trial containing:

- target ID and target-seed provenance;
- robot and scenario;
- solver and configuration hash;
- solver RNG seed;
- initial `q0`;
- target pose and, if available, source configuration;
- final `q`;
- success under each scorer;
- collision under each scorer;
- clean-success indicator;
- pose errors;
- clearances;
- latency;
- candidate/replica counts;
- phase taken;
- rescue counts and scopes;
- energy/FK/Jacobian evaluation counts;
- termination reason.

Aggregate tables must be generated from this immutable trial file.

## 6.2 Use paired analysis

Because solvers see identical targets, treat comparisons as paired.

- Use McNemar’s test or an exact paired test for paired binary success/clean-success outcomes.
- Use paired bootstrap confidence intervals for rate differences.
- Use paired permutation or bootstrap analysis for clearance and latency differences.
- Report confidence intervals for all headline proportions.
- Correct for multiple comparisons or define a small set of primary comparisons in advance.

Do not rely only on overlapping or non-overlapping marginal confidence intervals.

## 6.3 Separate target variability from solver stochasticity

There are two sources of variance:

1. which targets were drawn;
2. the random trajectory of a stochastic solver on a fixed target.

Use hierarchical repetition:

- multiple target seeds;
- multiple solver seeds per target for stochastic methods.

Analyze both target-level and run-level variability. A mixed-effects logistic model or hierarchical bootstrap can estimate whether gains generalize beyond the sampled target set.

## 6.4 Report counts, effects, and uncertainty

Every percentage table should provide at least one of:

- numerator/denominator;
- 95% confidence interval;
- paired difference with interval.

Prefer effect sizes over ratio rhetoric near zero. For example, at 16 DOF report `1/120 versus 0/120`, not an infinite ratio or “last solver standing.”

## 6.5 Power the main experiments

Choose sample sizes based on the smallest meaningful difference. Examples:

- If a 2 percentage-point success improvement matters near 95–99%, use enough targets to resolve it reliably.
- If a 5 percentage-point clean-success improvement matters, power the paired comparison around that effect.
- Increase the high-DOF sample substantially because clean successes are rare.

Do not choose `n=120` at 16 DOF if the expected clean rate is around 1%; that yields approximately one event and cannot support a strong conclusion.

---

## 7. Fairness and budget accounting

## 7.1 Define what “one solve” means

KineticFold’s single invocation contains multiple replicas, escalation, and candidate selection. Call this one solver invocation, not one optimization attempt.

Report:

- number of local solves;
- number of Phase-B runs;
- total iterations;
- FK/Jacobian evaluations;
- energy evaluations;
- total candidate configurations considered;
- wall-clock time.

## 7.2 Compare under two budget regimes

Run both:

1. **Native/default regime:** each method uses its documented normal configuration.
2. **Budget-matched regime:** equal wall-clock, equal pose-evaluation count, or equal candidate count.

The default regime answers “which tool works best as configured?” The matched regime answers “is the proposed search strategy more efficient?” Both are valuable and should not be conflated.

## 7.3 Separate algorithmic speed from implementation speed

Native C++ is a legitimate implementation contribution, but compilation alone does not prove the schedule is superior.

Report:

- same-language/same-compiler comparisons where possible;
- algorithmic work counts independent of language;
- C++ versus Python parity separately from algorithm comparisons;
- build flags and compiler version;
- repeated timing runs with CPU affinity and controlled load.

## 7.4 Real-time claims

Only use “real-time capable” after specifying:

- required frequency/deadline;
- measured end-to-end latency;
- maximum or extreme-tail latency over a large run;
- deadline miss rate;
- hardware and OS;
- whether memory allocation, target conversion, collision model update, and communication are included.

Otherwise use “fast solver-core latency on the benchmark platform.”

---

## 8. Benchmark and code upgrades

## 8.1 New result directory structure

Create immutable, timestamped run directories:

```text
backend/results/paper_release_v1/
  config.json
  environment.json
  git.json
  targets.npz
  trials.parquet
  aggregates.csv
  statistics.json
  validation.json
  console.log
  checksums.sha256
  figures/
  tables/
```

Never overwrite a paper-grade run. A `LATEST_APPROVED` pointer can identify the accepted release.

## 8.2 Resume safely

The current manifests show that final files can contain cells from earlier invocations. Resuming is useful, but each trial/cell must record:

- code commit;
- config hash;
- environment hash;
- start and finish time;
- whether it was resumed;
- originating run ID.

The aggregator must refuse to merge incompatible hashes. Before submission, preferably execute one clean full run from an approved commit.

## 8.3 Freeze target sets

Generate targets once and save them. All solvers should consume the same serialized target artifact instead of regenerating them independently from matching seeds. Verify its checksum before each cell.

Maintain separate target files for warm-up, development, and final test.

## 8.4 Add phase instrumentation

KineticFold should return explicit diagnostics:

- `phase_a_attempts`;
- `phase_a_converged`;
- `phase_a_clean`;
- `phase_b_entered`;
- `phase_b_attempts`;
- accepted/rejected/uphill Metropolis moves;
- scoped versus global rescue counts;
- winning candidate source;
- number of converged candidates;
- clearance-selection gain over first converged;
- stability-gate outcome.

This makes statements such as “79% take the fast path” directly auditable from the result file rather than reconstructed separately.

## 8.5 Add clean metrics directly

The scorer should produce for each engine:

- success;
- collision;
- clean success;
- collision conditional on success;
- clearance conditional on success;
- penetration depth conditional on collision and success.

Do not ask plotting code to infer these from unrelated aggregate columns.

## 8.6 Improve collision validation

PyBullet and MuJoCo currently provide useful independent software calculations on the same URDF. Strengthen this by:

- recording URDF and mesh checksums;
- validating pair filters against the manufacturer’s allowed-collision matrix;
- documenting primitive versus mesh collision shapes;
- evaluating multiple safety margins, not only zero signed distance;
- reporting pair-specific collision frequencies;
- inspecting a stratified sample visually;
- adding a third geometry source or CAD-derived model if available;
- checking whether rankings persist at 2 mm, 5 mm, and 10 mm safety clearances.

## 8.7 Add manuscript consistency tests

Create automated tests that parse or generate every numerical claim from approved artifacts. Examples:

- figure solver lists match the declared field;
- the named winner is actually the minimum/maximum under the stated metric;
- all percentages match the source CSV;
- all sample sizes match the manifest;
- no table combines incompatible runs;
- captions identify scorer, denominator, and field;
- no “all solvers” claim omits eligible solvers;
- no ratio is emitted when the denominator is zero.

Ideally, prose numbers should be inserted from generated snippets rather than manually copied.

## 8.8 Testing requirements

Before accepting a paper release:

- all backend tests pass;
- native baseline smoke/parity tests pass in the benchmark environment;
- C++/Python solver parity is rerun and archived;
- all three robot FK parity tests pass;
- trial-count and unique-target tests pass;
- deterministic rerun checks pass where expected;
- stochastic distribution checks stay within predefined tolerances;
- figure and table generation completes from an empty output directory.

---

## 9. Literature and novelty upgrade

## 9.1 Expand collision-aware IK related work

The related-work section currently acknowledges BioIK but does not treat collision-aware IK as the necessary empirical comparison class. Add focused coverage of:

- constrained and multi-objective IK;
- self-collision-aware IK;
- CollisionIK;
- DawnIK;
- BioIK’s collision objectives;
- MoveIt/Drake-style constrained IK;
- null-space collision avoidance;
- trajectory optimization methods when relevant.

Explain how ProteinIK differs in objective, search schedule, single-pose versus trajectory behavior, and computational budget.

## 9.2 Search for algorithmic analogues, not just biological terminology

Novelty cannot be established by searching only for “protein folding IK.” Search for the underlying structure:

- algorithm cascades and portfolios;
- adaptive restarts;
- variable-neighborhood search;
- block-coordinate perturbation;
- local-to-global restart schedules;
- conditional simulated annealing;
- basin hopping;
- trust-region escalation;
- coarse-to-fine IK;
- random-restart IK with best-solution selection.

The likely novelty, if any, is a particular combination and gate—not the existence of its ingredients.

## 9.3 Treat the folding analogy accurately

Consult domain experts or authoritative sources for each biological mapping. Distinguish:

- inspiration;
- loose computational analogy;
- mechanistic correspondence;
- mathematically equivalent structure.

Do not claim that a target-blind neutral-pose relaxation is biologically equivalent to secondary-structure formation unless the mapping is carefully justified. Do not imply proteins know a target conformation in the way an IK solver receives a target pose.

## 9.4 Novelty decision gate

After the literature review, choose one claim level:

- **Level 1:** new application-inspired heuristic combination;
- **Level 2:** new conditional escalation schedule for collision-aware IK;
- **Level 3:** new scoped rescue mechanism with measured efficiency advantage;
- **Level 4:** validated general design principle derived from folding.

Do not claim Level 4 unless the ablations and held-out scaling tests directly support it. A strong Level 2 or Level 3 paper is better than an unsupported Level 4 paper.

---

## 10. Paper rewrite plan by section

## 10.1 Title

Candidate conservative title:

> **ProteinIK: A Protein-Folding-Inspired Cascade for Collision-Aware Inverse Kinematics**

Candidate mechanism-focused title:

> **Conditional Escalation and Scoped Rescue for Fast Collision-Aware Inverse Kinematics**

Use the second if the biological ablations do not establish a stronger folding-specific claim.

## 10.2 Abstract

The final abstract should contain only:

- problem;
- precise algorithm;
- benchmark scale;
- primary comparison field;
- one success result with uncertainty;
- one clean-success/collision result against a matched collision-aware baseline;
- one latency result with hardware context;
- one carefully bounded conclusion.

Remove “isomorphism,” “proves itself,” and endpoint ratios based on one event.

## 10.3 Introduction

Start from the practical gap: fast pose IK often ignores collision, while full constrained search can be expensive. Introduce folding as the design inspiration for a cascade that spends expensive search only on difficult targets.

State the hypothesis in falsifiable form:

> Conditional escalation and scoped rescue should reduce average work while preserving clean-solve probability on targets where local IK is trapped by collision constraints.

This is testable and does not depend on accepting the metaphor.

## 10.4 Related work

Organize by technical comparison class:

1. local numerical IK;
2. restart and portfolio IK;
3. constrained/collision-aware IK;
4. stochastic and annealing-based IK;
5. biologically inspired optimization;
6. robotics–protein kinematic connections.

End with a comparison table showing which methods have collision objectives, restarts, conditional escalation, local rescue, multiple candidate selection, trajectory support, and real-time evidence.

## 10.5 Methodology

Give each operation a technical name first and biological analogy second. For example:

- “neutral-pose coordinate relaxation (secondary-structure-inspired)”;
- “coarse DLS approach (collapse-inspired)”;
- “annealed collision-aware refinement (funnel-inspired)”;
- “sensitivity-guided variable-neighborhood restart (chaperone-inspired)”;
- “conditional two-stage compute allocation (kinetic-partitioning-inspired).”

Specify exact pseudocode, budgets, termination criteria, candidate selection, and complexity. Make it obvious that one invocation can contain multiple replicas.

## 10.6 Experiments

Separate:

- hypotheses;
- development versus held-out data;
- baseline implementations;
- fairness/budgets;
- metrics and denominators;
- statistics;
- hardware;
- validation models;
- preregistered primary comparisons.

Include the target and result artifact hashes.

## 10.7 Results

Present in this order:

1. primary matched comparison;
2. clean-success results;
3. component ablation;
4. latency/work analysis;
5. generalization;
6. scaling;
7. trajectory or deployment experiment;
8. simulator/geometry sensitivity;
9. negative results.

Do not lead with weak textbook baselines. They can remain for context but should not carry the contribution.

## 10.8 Discussion

Explicitly distinguish findings:

- supported;
- consistent with the folding hypothesis but not unique to it;
- unsupported or negative;
- still unknown.

Discuss whether the winning effect comes from collision information, candidate selection, conditional escalation, or scoped rescue.

## 10.9 Limitations

Retain and expand:

- simulation-only status if applicable;
- self-collision versus environmental collision;
- hand-tuned proxy geometry;
- internal ensemble cost;
- potential trajectory discontinuity;
- lack of guaranteed convergence or safety;
- dependence on reachable target generation;
- held-out generalization limits;
- inability to equate protein folding and IK physically.

## 10.10 Conclusion

Conclude with the measured technical result, not the metaphor. A suitable structure is:

> The study shows that a conditional, collision-aware cascade can allocate inexpensive local search to easy targets and reserve stochastic refinement for frustrated ones. Matched ablations identify which components contribute to speed and clean-solve rate. Protein-folding concepts provided the design decomposition; the resulting algorithm is best understood as a practical constrained-IK strategy rather than a physical model of protein folding.

---

## 11. Figure and table redesign

## 11.1 Mandatory figures

1. **Algorithm diagram:** technical operations and biological inspirations in parallel lanes.
2. **Success and clean-success:** paired confidence intervals, not only bars.
3. **Collision conditional on success:** both physics engines and safety margins.
4. **Latency/work distribution:** ECDF or violin/box plot plus p50/p95/p99/max.
5. **Ablation forest plot:** paired effect of each component.
6. **Scaling plot:** counts and confidence intervals at each DOF; no ratios at zero.
7. **Generalization plot:** development versus held-out robots.
8. **Failure taxonomy:** singular failure, pose failure, collision, timeout, instability.

## 11.2 Mandatory tables

1. Robot and collision-model details.
2. Solver implementation, objective awareness, internal candidates, and budgets.
3. Primary paired comparisons with confidence intervals.
4. Ablation results.
5. Reproducibility environment and artifact hashes.
6. Negative and null results.

## 11.3 Plotting rules

- Solver inclusion is configuration-driven, not hard-coded ad hoc.
- Captions state sample size, denominator, field, scorer, and uncertainty.
- Figures regenerate directly from approved trial-level data.
- Highlighting the proposed solver is acceptable, but omitted eligible solvers must be explained.
- Use absolute differences near zero instead of unstable ratios.

---

## 12. Reproducibility package

The release should include:

- a clean installation guide;
- pinned Python and native dependencies;
- container or reproducible WSL/Linux environment;
- one command to build native solvers;
- one quick smoke benchmark;
- one command for each paper experiment;
- immutable target datasets;
- trial-level outputs or a documented public archive;
- figure/table generation commands;
- checksums;
- expected runtime and disk requirements;
- license information for code, robot assets, and dependencies.

Run the complete reproduction once on a fresh machine or clean virtual environment before submission.

---

## 13. Red-team review checklist

Before freezing the paper, assign a reviewer—human or AI—to argue against every major claim.

For each sentence containing “first,” “novel,” “best,” “fastest,” “cleanest,” “all,” “proves,” “real-time,” “independent,” or “significant,” require:

- exact definition;
- eligible comparison set;
- source artifact;
- statistical support;
- known counterexample;
- revised wording if any condition fails.

Specific hostile-review questions:

1. Would collision-aware Multi-start reproduce the headline collision gain?
2. Is the method still better if the clearance-selection wrapper is removed?
3. Is gating useful after controlling for replica count and work?
4. Does scoped rescue beat global restart on the exact same objective and budget?
5. Does the biological interpretation predict a result not already predicted by standard optimization theory?
6. Are held-out robots genuinely untouched during tuning?
7. Does the advantage persist under CAD-derived geometry and safety margins?
8. Is the high-DOF result more than one successful event?
9. Does the solver generate continuous, executable trajectories?
10. Can every number be regenerated from one approved release artifact?

Do not submit until every question has either a satisfactory answer or an explicit limitation in the paper.

---

## 14. Upgrade phases and gates

### Phase 0 — Freeze the initial draft

Deliverables:

- initial draft declared complete by the owner;
- tagged repository state;
- archived current figures and result files.

Gate: only after this point may this `upgrade.md` plan be used.

### Phase 1 — Factual repair

Deliverables:

- corrected solver field;
- corrected collision denominator;
- corrected warm-up description/code;
- corrected Raw/Langevin status;
- corrected latency and 16-DOF wording.

Gate: manuscript contains no claim contradicted by its authoritative artifacts.

### Phase 2 — Benchmark infrastructure

Deliverables:

- trial-level schema;
- immutable target sets;
- safe resume/provenance;
- phase diagnostics;
- generated clean metrics;
- manuscript consistency tests.

Gate: a quick run regenerates all aggregate outputs and passes consistency tests.

### Phase 3 — Matched baselines and ablations

Deliverables:

- clearance-selecting Multi-start;
- matched LM cascade;
- non-biological annealing baseline;
- scoped/global rescue comparison;
- full component ablation;
- at least one established collision-aware baseline or documented best feasible substitute.

Gate: the central technical contribution remains useful against objective- and budget-matched alternatives.

### Phase 4 — Generalization and statistics

Deliverables:

- held-out robot evaluation;
- powered DOF scaling;
- repeated solver seeds;
- paired statistical analysis;
- confidence intervals and effect sizes.

Gate: primary gains are not dependent on one robot, one target draw, or one stochastic run.

### Phase 5 — Deployment evidence

Deliverables:

- trajectory-level experiment;
- end-to-end latency;
- hardware or hardware-in-the-loop validation where feasible;
- safety-margin sensitivity.

Gate: deployment language matches actual evidence.

### Phase 6 — Final rewrite and release

Deliverables:

- revised paper;
- regenerated figures/tables;
- artifact package;
- clean-environment reproduction;
- red-team report;
- final claim ledger.

Gate: every headline claim is automatically traceable and survives red-team review.

---

## 15. Claim ledger template

Maintain a machine-readable or Markdown claim ledger during the upgrade:

| ID | Draft claim | Claim type | Exact metric | Eligible field | Artifact | Statistical test | Result | Final wording |
|---|---|---|---|---|---|---|---|---|
| C1 | KineticFold improves hard-cell success | Quantitative | Paired success difference | Core field | release trials | McNemar + paired CI | TBD | TBD |
| C2 | KineticFold improves clean success | Quantitative | `P(success ∧ clean)` | Collision-aware field | release trials | Paired bootstrap/test | TBD | TBD |
| C3 | Gating reduces latency | Mechanistic | Paired time/work difference | Gate ablation | ablation trials | Paired bootstrap | TBD | TBD |
| C4 | Scoped rescue beats global restart | Mechanistic | Clean success per work | Rescue ablation | ablation trials | Paired test | TBD | TBD |
| C5 | Advantage grows with chain length | Trend | Difference, not ratio | Scaling field | scaling trials | Trend model/interaction | TBD | TBD |
| C6 | Folding concepts predict useful organization | Interpretive | Ablation pattern | Matched variants | combined | Predefined qualitative criterion | TBD | TBD |

No claim enters the abstract or conclusion unless its ledger row is complete.

---

## 16. Possible outcomes and how to write them honestly

### Outcome A: all major components survive

If gating, scoped rescue, and collision-aware annealing each provide independent gains against matched baselines, the paper can make a strong algorithmic and design-principle claim.

### Outcome B: clearance selection explains most collision gains

Then reposition KineticFold as an efficient internal candidate-generation and selection strategy. The biological story becomes secondary. This can still be useful research.

### Outcome C: collision energy explains the gains

Then the correct finding is that adding explicit collision awareness matters; ProteinIK’s specific process may not. Report the null ablations and narrow the contribution.

### Outcome D: conditional gating explains speed but not quality

Then publish the cascade as a compute-allocation method: a fast local front end with an expensive reliability fallback.

### Outcome E: established collision-aware solvers win

Then the paper can become an honest comparative/negative study showing where folding-inspired heuristics help and where conventional constrained optimization is superior. Do not hide this result.

### Outcome F: no robust advantage remains

The project still provides a valuable benchmark, simulator-validation pipeline, and case study in translating biological metaphors into algorithms. The conclusion must change, but the work need not be discarded.

---

## 17. Definition of “best possible paper”

The upgraded paper is ready when all of the following are true:

- No designated result contradicts the prose.
- Solver fields and exclusions are explicit.
- Clean success is measured as a joint event.
- The central collision comparison includes a collision-aware matched baseline.
- Internal replicas and clearance selection are counted transparently.
- Gating, annealing, rescue, selection, and collision cost are separately ablated.
- Primary comparisons use paired uncertainty and tests.
- High-DOF claims are powered by enough events.
- Held-out robots demonstrate generalization.
- Timing includes hardware details and avoids unsupported real-time language.
- Simulator validation is described as software cross-validation unless hardware evidence exists.
- Every figure and table regenerates from immutable trial-level artifacts.
- A clean machine can reproduce the approved release.
- The biological framing is presented as strongly as the evidence permits, but no stronger.

The project’s opportunity is genuine. The strongest future version will not depend on persuading reviewers that robot arms literally fold like proteins. It will demonstrate that a clearly specified, folding-inspired compute strategy solves constrained IK efficiently, show exactly which components matter, and remain valuable even under a completely non-biological interpretation.
