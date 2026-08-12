# IMPORTANT — DEFERRED CHECKLIST: DO NOT USE YET

> **DO NOT CONSIDER, APPLY, CITE, OR USE THIS CHECKLIST WHILE COMPLETING THE INITIAL DRAFT.**
>
> This checklist is reserved for the later upgrade stage. Any AI, agent, reviewer, or contributor must ignore it until the project owner explicitly declares the initial draft complete and explicitly asks to begin the upgrade. Its existence is not permission to change the current draft, code, benchmarks, figures, or claims.

Related roadmap: [`upgrade-plan.md`](upgrade-plan.md)

---

# ProteinIK complete research-paper upgrade checklist

## How to use this checklist

- [ ] Do not start any item until the activation gate below is complete.
- [ ] Assign one owner to every activated section.
- [ ] Record evidence beside each completed item: commit, result directory, figure, table, test log, or review note.
- [ ] Treat `[ ]` as incomplete, `[~]` as in progress, `[x]` as verified, and `[N/A]` as deliberately excluded with a written reason.
- [ ] Never mark an experimental task complete merely because the code runs; require saved results and validation.
- [ ] Never mark a manuscript task complete merely because wording changed; require agreement with the approved artifacts.
- [ ] Reopen an item if later results invalidate it.
- [ ] Keep the checklist on a paper-release branch or milestone after activation.

Recommended evidence format:

```text
Owner:
Completed date:
Commit/config hash:
Evidence path or URL:
Reviewer:
Notes:
```

## 0. Activation gate — mandatory

- [ ] Project owner explicitly states: “The initial draft is complete.”
- [ ] Project owner explicitly states: “Begin the deferred upgrade plan and checklist.”
- [ ] Save the owner’s authorization in the project log.
- [ ] Tag or archive the exact initial-draft repository state.
- [ ] Archive the initial draft, figures, benchmark tables, manifests, and result CSVs without modification.
- [ ] Record the initial-draft Git commit and dirty-worktree status.
- [ ] Confirm that future edits occur on a separate branch or clearly named upgrade state.
- [ ] Assign an upgrade coordinator responsible for gates and evidence.

**Gate 0 passes only when every item above is verified.**

---

## 1. Establish the paper’s source of truth

### 1.1 Authoritative inputs

- [ ] Confirm the latest manuscript path.
- [ ] Confirm the authoritative success/latency result file.
- [ ] Confirm the authoritative collision result file.
- [ ] Confirm the authoritative result manifests.
- [ ] Confirm the authoritative ProteinIK/KineticFold implementation.
- [ ] Confirm the authoritative native baseline adapters.
- [ ] List every older or superseded result artifact.
- [ ] Ensure superseded artifacts cannot silently feed paper figures or tables.
- [ ] Create one machine-readable paper-release configuration.

### 1.2 Release configuration

- [ ] Record Git commit hash.
- [ ] Record whether the worktree is clean.
- [ ] Record operating system and version.
- [ ] Record native Windows, WSL, container, or VM status.
- [ ] Record CPU model, physical/logical core counts, RAM, and relevant power mode.
- [ ] Record compiler name and version.
- [ ] Record compiler flags, optimization level, and architecture flags.
- [ ] Record Python version and package lock/checksums.
- [ ] Record all native dependency versions.
- [ ] Record solver names and exact configuration hashes.
- [ ] Record robot-model names and URDF/mesh checksums.
- [ ] Record scenario definitions and hashes.
- [ ] Record target-set files and checksums.
- [ ] Record warm-up-set files and checksums.
- [ ] Record all random seeds.
- [ ] Record trial counts and repetition structure.
- [ ] Record timeouts, iteration limits, and convergence tolerances.
- [ ] Record collision-pair filters and safety margins.
- [ ] Record scorer versions and result-schema version.
- [ ] Make benchmark execution refuse an incomplete release configuration.

**Evidence:** approved release-config file and validation log.

---

## 2. Freeze the scientific questions before new experiments

- [ ] Write the primary research question for pose success.
- [ ] Write the primary research question for clean success.
- [ ] Write the primary research question for latency and computational work.
- [ ] Write the primary research question for conditional gating.
- [ ] Write the primary research question for scoped rescue.
- [ ] Write the primary research question for chain-length scaling.
- [ ] Write the primary research question for held-out generalization.
- [ ] Separate confirmatory questions from exploratory questions.
- [ ] Define one primary metric per confirmatory question.
- [ ] Define the smallest practically meaningful effect for each primary metric.
- [ ] Define pass, fail, and inconclusive criteria before inspecting new final-test results.
- [ ] Predefine all primary solver comparisons.
- [ ] Predefine eligible solver fields and exclusion rules.
- [ ] Predefine statistical tests and multiplicity correction.
- [ ] Freeze the experimental specification with a timestamp and commit hash.
- [ ] Ensure held-out targets and robots remain unopened until methods are frozen.

**Gate 1:** no final benchmark begins until the frozen experimental specification exists.

---

## 3. Repair known factual problems

### 3.1 Solver-field and “cleanest” claims

- [ ] Recompute collision rankings from the designated CSV.
- [ ] Explicitly account for `protein_raw`/LangevinFold.
- [ ] Choose and document whether Raw is a main solver, extended solver, or excluded method.
- [ ] If excluded, define the exclusion criterion before presenting rankings.
- [ ] Remove any “cleanest of any solver” wording contradicted by Raw.
- [ ] Ensure plotting code does not hide eligible solvers through a hand-written list.
- [ ] Define `core_field`, `collision_aware_field`, `extended_field`, and `diagnostic_variants` or equivalent.
- [ ] Name the applicable solver field in every comparison.
- [ ] Search the manuscript for “all solvers,” “whole field,” “any solver,” “best,” and “cleanest.”
- [ ] Verify each such phrase literally matches the declared eligible field.

### 3.2 Collision denominators

- [ ] Verify in code whether failures are included in the collision denominator.
- [ ] Remove the false statement that failed hard targets are excluded, if still present.
- [ ] Define `success_rate = P(success)`.
- [ ] Define `collision_given_success = P(collision | success)`.
- [ ] Define `clean_success_rate = P(success AND collision_free)`.
- [ ] Compute clean success per trial as a joint event.
- [ ] Prohibit estimating clean success by multiplying aggregate percentages.
- [ ] State denominators in collision table and figure captions.
- [ ] Provide numerator and denominator for every headline collision percentage.

### 3.3 Warm-up procedure

- [ ] Inspect whether timed targets are reused for warm-up.
- [ ] Generate a disjoint warm-up target set.
- [ ] Save and checksum the warm-up target set.
- [ ] Verify warm-up target IDs cannot occur in timed evaluation.
- [ ] Update manuscript wording to exactly match the implementation.
- [ ] Add an automated overlap test.

### 3.4 Latency claims

- [ ] Recompute mean latency.
- [ ] Recompute median latency.
- [ ] Recompute p90, p95, and p99 latency.
- [ ] Recompute maximum latency.
- [ ] Report deadline-miss rates at relevant budgets.
- [ ] Define whether failed solves are included in timing summaries.
- [ ] Define whether low-success methods are eligible for timing rankings.
- [ ] Remove unsupported “smallest tail,” “fastest,” or “real-time” wording.
- [ ] Replace hard real-time language with platform-bounded measurements unless deadlines are tested.
- [ ] Report solver-core and end-to-end latency separately.

### 3.5 High-DOF claims

- [ ] Report 16-DOF results as counts, not only percentages.
- [ ] Replace ratios with absolute differences when a baseline has zero successes.
- [ ] Remove “last solver standing” or similar rhetoric based on one event.
- [ ] Increase the sample until the endpoint has enough events for a meaningful interval, or label it inconclusive.
- [ ] Add confidence intervals at every chain length.
- [ ] Test a predefined trend or interaction rather than narrating isolated endpoints.

### 3.6 Validation wording

- [ ] Describe PyBullet/MuJoCo comparison as independent software-engine cross-validation.
- [ ] State that both engines share robot description assumptions where applicable.
- [ ] Do not call simulator agreement physical validation.
- [ ] Use “single solver invocation” instead of “single-shot” when internal replicas exist.
- [ ] Replace “structural isomorphism” with “structural analogy” unless equivalence is formally established.
- [ ] Remove claims that proteins use the exact implemented computational sequence.

**Gate 2:** every numerical and methodological statement in the current manuscript agrees with its cited artifact.

---

## 4. Create and maintain a claim ledger

- [ ] Give every abstract claim a unique ID.
- [ ] Give every contribution claim a unique ID.
- [ ] Give every result superlative a unique ID.
- [ ] Give every novelty/first claim a unique ID.
- [ ] Give every causal/mechanistic claim a unique ID.
- [ ] Record exact claim wording.
- [ ] Record claim type: descriptive, comparative, mechanistic, causal, novelty, or interpretive.
- [ ] Record metric and denominator.
- [ ] Record eligible solver field.
- [ ] Record source artifact and checksum.
- [ ] Record sample size.
- [ ] Record statistical test and confidence interval.
- [ ] Record known counterexamples or limitations.
- [ ] Record final supported wording.
- [ ] Block claims with missing evidence from the abstract and conclusion.
- [ ] Generate manuscript numbers from the ledger or approved data where feasible.
- [ ] Have an independent reviewer sign off each headline ledger row.

---

## 5. Build fair matched baselines

### 5.1 Common rules for all baselines

- [ ] Use identical serialized target sets.
- [ ] Use identical initial configurations where applicable.
- [ ] Use common random numbers where applicable.
- [ ] Use identical pose tolerances.
- [ ] Use identical joint limits.
- [ ] Use the same collision model and safety margin for collision-aware comparisons.
- [ ] Record internal attempts, candidates, and objective evaluations.
- [ ] Record preprocessing separately from timed solving.
- [ ] Verify native adapters call the intended production algorithms.
- [ ] Document all wrapper behavior and post-selection.
- [ ] Tune baselines with a declared fair tuning protocol.
- [ ] Freeze baseline settings before held-out evaluation.

### 5.2 Matched LM cascade

- [ ] Implement the same Phase-A replica structure without folding-inspired Phase B.
- [ ] Match seeds and computational budget.
- [ ] Add a continuation using additional LM/global restarts.
- [ ] Report success, clean success, time, and work.
- [ ] Test whether conditional multi-start alone explains the result.

### 5.3 Annealed IK without scoped chaperone logic

- [ ] Use the same pose, limit, smoothness, and collision objective.
- [ ] Use the same Metropolis temperature schedule.
- [ ] Replace scoped rescue with ordinary global restart.
- [ ] Match time and objective-evaluation budgets.
- [ ] Compare success per unit work and clean success per unit work.

### 5.4 Fixed two-stage cascade

- [ ] Implement a variant that always executes both phases.
- [ ] Keep all other settings identical.
- [ ] Measure whether gating saves work.
- [ ] Measure whether gating loses difficult successes.

### 5.5 Clearance-selecting multi-start

- [ ] Wrap TRAC-IK or LM in repeated candidate generation.
- [ ] Match KineticFold’s candidate-count budget.
- [ ] Add a separate wall-clock-matched condition.
- [ ] Score candidates with the same clearance proxy.
- [ ] Select maximum-clearance successful candidate using identical logic.
- [ ] Report first-converged versus selected-candidate performance.
- [ ] Attribute collision gains to selection when supported.

### 5.6 Conventional collision-penalized optimizer

- [ ] Choose a standard optimizer suited to the objective.
- [ ] Use the same pose, joint-limit, regularization, and collision terms.
- [ ] Document gradients, numerical derivatives, and termination rules.
- [ ] Tune under the same development budget.
- [ ] Compare under candidate-count and wall-clock budgets.

### 5.7 Established collision-aware IK

- [ ] Evaluate feasibility of BioIK with collision objectives.
- [ ] Evaluate feasibility of CollisionIK.
- [ ] Evaluate feasibility of DawnIK.
- [ ] Evaluate feasibility of MoveIt or Drake constrained IK.
- [ ] Select at least one appropriate peer-reviewed/open collision-aware baseline.
- [ ] If integration is impossible, document exact technical incompatibilities.
- [ ] Implement the strongest feasible substitute when needed.
- [ ] Avoid presenting pose-only TRAC-IK as the sole collision comparison.

**Gate 3:** at least one objective-matched collision-aware baseline and one budget-matched selection baseline are operational and validated.

---

## 6. Run the full component ablation

For every variant, preserve identical targets and record all primary metrics and work counters.

- [ ] Frustration gate on versus both phases always run.
- [ ] Collision term current weight versus zero.
- [ ] Collision weight sensitivity sweep.
- [ ] Metropolis uphill acceptance versus greedy acceptance.
- [ ] Scoped local-to-global rescue versus immediate global restart.
- [ ] Maximum-clearance selection versus first converged.
- [ ] Maximum-clearance selection versus minimum-energy selection.
- [ ] Phase-A replicas: one, two, current count, and budget-matched alternative.
- [ ] Target-blind stage on versus off, if methodologically compatible.
- [ ] Difficulty-adaptive budget versus fixed budget.
- [ ] Stability gate on versus off.
- [ ] Folding-named regularizers versus conventional posture regularizers.
- [ ] Neutral-pose term sensitivity.
- [ ] Smoothness term sensitivity.
- [ ] Rescue-scope schedule sensitivity.
- [ ] Temperature schedule sensitivity.
- [ ] Termination-tolerance sensitivity.
- [ ] Test key interactions among gate, replica count, collision term, and selection.
- [ ] Use factorial or fractional-factorial design for interactions where feasible.
- [ ] Report null and negative ablations.
- [ ] Avoid selecting only favorable ablation rows for the paper.

For each row report:

- [ ] Pose success.
- [ ] Collision rate over all returns.
- [ ] Collision conditional on success.
- [ ] Clean-success rate.
- [ ] Clearance conditional on success.
- [ ] Penetration depth for colliding successes.
- [ ] Mean, median, p95, p99, and maximum time.
- [ ] Candidate and replica counts.
- [ ] FK, Jacobian, and energy evaluation counts.
- [ ] Phase-entry and rescue counts.
- [ ] Paired effect size and uncertainty.

**Gate 4:** the paper’s named mechanism is supported by matched ablations, or the contribution is honestly reframed.

---

## 7. Expand target and scenario coverage

### 7.1 Target construction

- [ ] Preserve reachable-by-construction targets for comparability.
- [ ] Save exact target pose, source configuration, and target ID.
- [ ] Verify target uniqueness.
- [ ] Verify target reachability under the scoring model.
- [ ] Classify whether at least one clean solution is known.
- [ ] Use a high-budget oracle or large multi-start procedure to estimate clean feasibility.
- [ ] Separate targets with no known clean solution from algorithm failures.

### 7.2 Required scenario families

- [ ] Uniform reachable poses.
- [ ] Near-singular targets.
- [ ] Self-collision-prone target poses.
- [ ] Difficult/colliding initial configurations with ordinary targets.
- [ ] Large Cartesian displacement.
- [ ] Large orientation displacement.
- [ ] Near-joint-limit targets.
- [ ] Targets with multiple IK branches.
- [ ] Targets with both clean and colliding solutions.
- [ ] Targets with no known clean solution.
- [ ] Workspace-obstacle scenarios if obstacle claims are retained or added.
- [ ] Safety-margin sweeps.
- [ ] Adversarial or stress-test scenarios clearly labeled exploratory.

### 7.3 Scenario validation

- [ ] Define scenario rules without using final solver outcomes.
- [ ] Check scenario balance and sample sizes.
- [ ] Plot target distributions in pose/joint space.
- [ ] Detect duplicate or near-duplicate targets.
- [ ] Ensure warm-up, development, and final sets are disjoint.
- [ ] Archive target-generation code and logs.

---

## 8. Demonstrate generalization

- [ ] Declare which robots and scenarios are development data.
- [ ] Declare which robots and scenarios are held out.
- [ ] Include variation in DOF.
- [ ] Include variation in redundancy.
- [ ] Include variation in joint-limit asymmetry.
- [ ] Include variation in link geometry and scale.
- [ ] Add at least one unseen 6-DOF industrial arm if feasible.
- [ ] Add at least one unseen 7-DOF redundant arm if feasible.
- [ ] Add an unseen hyper-redundant synthetic chain if scaling is a claim.
- [ ] Freeze ProteinIK weights before opening held-out outcomes.
- [ ] Derive robot-specific radii/thresholds mechanically rather than outcome tuning.
- [ ] Log any post-freeze change and rerun all held-out comparisons if necessary.
- [ ] Report development and held-out results separately.
- [ ] Test robot-by-method interactions.
- [ ] State exactly where generalization fails.

**Gate 5:** primary gains are not dependent on one tuned robot or one target distribution.

---

## 9. Add trajectory and deployment evidence

### 9.1 Continuous trajectories

- [ ] Define continuous target sequences.
- [ ] Warm-start each solve from the previous solution in a dedicated condition.
- [ ] Measure end-effector position and orientation error over time.
- [ ] Count self-collision events.
- [ ] Measure minimum clearance over time.
- [ ] Measure joint velocity.
- [ ] Measure joint acceleration.
- [ ] Measure joint jerk.
- [ ] Measure joint-space discontinuities and branch switches.
- [ ] Measure solve-deadline misses.
- [ ] Test temporary infeasibility and recovery.
- [ ] Compare first-converged and maximum-clearance selection for continuity.
- [ ] Add displacement/smoothness-aware candidate selection if justified.
- [ ] Limit claims to pose IK/planning if servo continuity is poor.

### 9.2 Hardware or hardware-in-the-loop

- [ ] Decide whether physical hardware is feasible and safe.
- [ ] Obtain necessary supervision and safety approval.
- [ ] Use conservative speed, clearance, and joint-limit margins.
- [ ] Define safe target poses before deployment.
- [ ] Measure actual end-effector error with calibrated sensing if available.
- [ ] Confirm controller-level joint-limit compliance.
- [ ] Measure full perception-to-command or target-to-command latency.
- [ ] Record controller rejection and safety-stop events.
- [ ] Record all failures, not only successful demonstrations.
- [ ] Publish representative videos/logs when permitted.
- [ ] If hardware is unavailable, run hardware-in-the-loop or a full control stack.
- [ ] Retain “simulation-only” as a limitation if no physical experiment exists.

**Gate 6:** deployment wording is no stronger than the actual trajectory/hardware evidence.

---

## 10. Upgrade trial-level data collection

Each trial record must include:

- [ ] Stable run ID and trial ID.
- [ ] Target ID and target-set checksum.
- [ ] Target-generation seed and source provenance.
- [ ] Robot and scenario ID.
- [ ] Solver and configuration hash.
- [ ] Solver RNG seed.
- [ ] Initial joint configuration `q0`.
- [ ] Target position and orientation.
- [ ] Source joint configuration when applicable.
- [ ] Returned joint configuration.
- [ ] Termination reason.
- [ ] Success under the primary scorer.
- [ ] Success under every cross-validation scorer.
- [ ] Collision under the primary collision engine.
- [ ] Collision under every cross-validation engine.
- [ ] Clean-success indicator.
- [ ] Position and orientation errors.
- [ ] Minimum clearance and colliding pair IDs.
- [ ] Penetration depth where available.
- [ ] Solver-core latency.
- [ ] End-to-end latency where applicable.
- [ ] Candidate and replica counts.
- [ ] Phase entered and winning phase.
- [ ] Rescue count and scope.
- [ ] Accepted, rejected, and uphill stochastic moves.
- [ ] FK, Jacobian, collision, and energy evaluation counts.
- [ ] Winning candidate source.
- [ ] Number of converged candidates.
- [ ] Clearance gain over first converged.
- [ ] Stability-gate outcome.
- [ ] Git commit, environment hash, and result-schema version.
- [ ] Resumed/original status and originating run ID.

Data integrity checks:

- [ ] Enforce schema validation when writing results.
- [ ] Reject missing required fields.
- [ ] Reject incompatible configuration hashes during aggregation.
- [ ] Reject duplicate trial keys.
- [ ] Verify expected trial count for every cell.
- [ ] Verify all compared solvers received identical target IDs.
- [ ] Make aggregate tables reproducible from immutable trial rows.
- [ ] Retain raw rows even when a trial fails or crashes.

---

## 11. Statistical analysis

### 11.1 Study design and sample size

- [ ] Define smallest meaningful success-rate difference.
- [ ] Define smallest meaningful clean-success difference.
- [ ] Define smallest meaningful latency/work difference.
- [ ] Perform power or precision analysis for primary comparisons.
- [ ] Increase sample sizes for rare high-DOF events.
- [ ] Predefine stopping rules.
- [ ] Prohibit optional stopping based on favorable interim results.

### 11.2 Paired inference

- [ ] Use paired target outcomes for solver comparisons.
- [ ] Use McNemar’s or an exact paired test for binary outcomes.
- [ ] Use paired bootstrap intervals for rate differences.
- [ ] Use paired permutation/bootstrap methods for latency and clearance.
- [ ] Report absolute effects, not only ratios.
- [ ] Report 95% confidence intervals for headline proportions and differences.
- [ ] Define and apply multiplicity correction, or restrict confirmatory tests in advance.
- [ ] Distinguish statistical significance from practical significance.

### 11.3 Hierarchical variability

- [ ] Run multiple target seeds.
- [ ] Run multiple solver seeds per target for stochastic methods.
- [ ] Separate target-sampling variance from solver-stochasticity variance.
- [ ] Use a hierarchical bootstrap or mixed-effects model where appropriate.
- [ ] Treat target as a paired/random effect.
- [ ] Test robustness to seed choice.
- [ ] Avoid treating repeated solver seeds on one target as independent targets.

### 11.4 Reporting

- [ ] Report numerator/denominator for percentages.
- [ ] Report effect size and confidence interval.
- [ ] Report exact test or model.
- [ ] Report handling of timeouts and crashes.
- [ ] Report handling of ties and missing values.
- [ ] Report exploratory analyses as exploratory.
- [ ] Archive analysis scripts and rendered outputs.
- [ ] Have a second reviewer reproduce at least the primary statistics.

**Gate 7:** every headline comparison has a prespecified metric, paired effect, uncertainty estimate, and sufficient events.

---

## 12. Computational fairness and budget accounting

- [ ] Define what counts as one solver invocation.
- [ ] State that KineticFold may use multiple internal replicas/candidates.
- [ ] Count all internal work, including rejected candidates and rescue attempts.
- [ ] Separate initialization/preprocessing from per-target solve time.
- [ ] Measure candidate count.
- [ ] Measure objective/energy evaluations.
- [ ] Measure FK and Jacobian evaluations.
- [ ] Measure collision checks.
- [ ] Measure iterations and restarts.
- [ ] Measure CPU time and wall-clock time.
- [ ] Record thread count and parallelism.
- [ ] Prevent accidental multi-thread advantage unless declared.
- [ ] Run an equal candidate-count comparison.
- [ ] Run an equal wall-clock-budget comparison.
- [ ] Run an equal objective-evaluation comparison where feasible.
- [ ] Run a default-settings practical comparison separately.
- [ ] Distinguish algorithmic differences from language/implementation differences.
- [ ] If Python and C++ implementations differ, report that limitation or add parity.
- [ ] Avoid ranking latency among methods that almost always fail without qualification.
- [ ] Report accuracy/cleanliness versus compute Pareto curves.

---

## 13. Benchmark engineering and provenance

### 13.1 Immutable release directories

- [ ] Create a unique run directory per release execution.
- [ ] Save resolved configuration.
- [ ] Save environment and dependency metadata.
- [ ] Save target and warm-up checksums.
- [ ] Save stdout/stderr and test logs.
- [ ] Save trial-level records.
- [ ] Save generated aggregates.
- [ ] Save figures and tables.
- [ ] Save a manifest connecting every artifact.
- [ ] Make approved release directories read-only or content-addressed.

### 13.2 Safe resume

- [ ] Record commit/config/environment hash per cell or trial.
- [ ] Record start/end timestamps and originating invocation.
- [ ] Record whether data was resumed.
- [ ] Refuse to merge incompatible hashes.
- [ ] Refuse to overwrite completed approved cells silently.
- [ ] Test interruption and resume behavior.
- [ ] Perform one clean, non-composite final run where feasible.
- [ ] If a composite run remains, disclose and prove compatibility.

### 13.3 Automated consistency checks

- [ ] Check manuscript percentages against approved data.
- [ ] Check sample sizes against manifests.
- [ ] Check solver lists against declared fields.
- [ ] Check superlatives against actual rankings.
- [ ] Check every caption for metric, denominator, field, scorer, and `n`.
- [ ] Block ratios with zero denominators.
- [ ] Block mixing incompatible run hashes.
- [ ] Block use of superseded artifacts.
- [ ] Generate tables and numeric snippets from source data.
- [ ] Run checks in CI.

### 13.4 Test suite

- [ ] All backend unit/integration tests pass.
- [ ] Native baseline smoke tests pass.
- [ ] Native baseline parity tests pass in the benchmark environment.
- [ ] C++/Python solver parity is rerun and archived.
- [ ] FK parity passes for every robot.
- [ ] Collision-engine sanity tests pass.
- [ ] Trial-count and unique-target tests pass.
- [ ] Warm-up-disjointness test passes.
- [ ] Deterministic rerun checks pass where expected.
- [ ] Stochastic distribution checks meet predefined tolerances.
- [ ] Figure/table generation succeeds from an empty output directory.
- [ ] Fresh-environment smoke reproduction passes.

**Gate 8:** one command can reproduce validated aggregates, figures, and tables from approved immutable trial data.

---

## 14. Strengthen collision validation

- [ ] Record URDF checksum.
- [ ] Record every collision-mesh checksum.
- [ ] Document primitive versus mesh geometry.
- [ ] Verify allowed-collision/pair-filter rules.
- [ ] Compare filters with manufacturer or canonical robot configuration where available.
- [ ] Record per-pair collision frequencies.
- [ ] Inspect frequently colliding pairs for modeling artifacts.
- [ ] Inspect a stratified sample visually.
- [ ] Save screenshots or scene files for audited samples.
- [ ] Cross-check with PyBullet.
- [ ] Cross-check with MuJoCo.
- [ ] Investigate every material disagreement between engines.
- [ ] Evaluate 0 mm clearance and nonzero safety margins such as 2, 5, and 10 mm.
- [ ] Report whether solver rankings persist across margins.
- [ ] Add CAD-derived geometry or a third source if feasible.
- [ ] Distinguish self-collision from environmental collision.
- [ ] Do not infer physical safety solely from simulator agreement.

---

## 15. Literature and novelty review

### 15.1 Search scope

- [ ] Search local numerical IK.
- [ ] Search random-restart and portfolio IK.
- [ ] Search constrained and multi-objective IK.
- [ ] Search self-collision-aware IK.
- [ ] Search BioIK collision objectives.
- [ ] Search CollisionIK.
- [ ] Search DawnIK.
- [ ] Search MoveIt/Drake constrained IK.
- [ ] Search null-space collision avoidance.
- [ ] Search stochastic/annealing-based IK.
- [ ] Search basin hopping and variable-neighborhood search.
- [ ] Search conditional computation/cascades.
- [ ] Search adaptive and scoped restarts.
- [ ] Search coarse-to-fine IK.
- [ ] Search best-candidate/clearance selection.
- [ ] Search biologically inspired optimization.
- [ ] Search robotics–protein kinematic connections.
- [ ] Search underlying algorithmic ideas, not only “protein folding IK.”

### 15.2 Source quality and citation verification

- [ ] Prefer primary papers and official implementations.
- [ ] Read every cited source sufficiently to verify the attributed claim.
- [ ] Verify author, title, venue, year, DOI, and URL.
- [ ] Remove citations that do not support the sentence.
- [ ] Avoid citing abstracts/snippets as proof when the full work is available.
- [ ] Distinguish peer-reviewed work from preprints and software pages.
- [ ] Build a comparison table of objectives, collision awareness, restarts, gating, rescue, candidate selection, and trajectory evidence.
- [ ] Ask a robotics expert to inspect missing baseline classes if possible.
- [ ] Ask a protein-folding expert to inspect biological mappings if possible.

### 15.3 Novelty decision

- [ ] Identify which components are known individually.
- [ ] Identify whether the combination is new.
- [ ] Identify whether the gating rule is new.
- [ ] Identify whether scoped rescue is new.
- [ ] Identify whether the biological decomposition predicts unique behavior.
- [ ] Select the justified novelty level from heuristic combination through validated design principle.
- [ ] Remove “first” or “novel” unless the search and evidence support it.
- [ ] Reframe to the strongest defensible technical contribution.

**Gate 9:** novelty wording survives a dedicated prior-art challenge and does not depend on searching only biological terminology.

---

## 16. Rewrite the manuscript

### 16.1 Title

- [ ] Use a conservative folding-inspired title if biology is secondary.
- [ ] Use a mechanism-focused title if ablations do not establish folding-specific value.
- [ ] Avoid title claims broader than evaluated collision type or task.

### 16.2 Abstract

- [ ] State the practical problem.
- [ ] Describe the algorithm technically and concisely.
- [ ] State benchmark scale and held-out scope.
- [ ] Name the relevant comparison field.
- [ ] Include one success result with uncertainty.
- [ ] Include one clean-success result against a matched collision-aware baseline.
- [ ] Include one latency/work result with hardware context.
- [ ] State a bounded conclusion.
- [ ] Remove unsupported isomorphism, proof, real-time, and dramatic endpoint claims.
- [ ] Verify every abstract number through the claim ledger.

### 16.3 Introduction and contributions

- [ ] Motivate the gap between fast pose IK and collision-aware constrained search.
- [ ] Present folding as design inspiration, not established physical equivalence.
- [ ] State a falsifiable conditional-escalation hypothesis.
- [ ] List contributions that correspond to tested evidence.
- [ ] Separate algorithm, evaluation, and design-inspiration contributions.
- [ ] Avoid claiming contributions that are merely implementation details unless independently valuable.

### 16.4 Related work

- [ ] Organize by technical comparison class.
- [ ] Include collision-aware IK as a central class.
- [ ] Include stochastic/restart/cascade analogues.
- [ ] Include biologically inspired optimization carefully.
- [ ] State similarities and differences without straw-manning baselines.
- [ ] End with an evidence-backed comparison table.

### 16.5 Method

- [ ] Give every operation a technical name first.
- [ ] Put the biological analogy second.
- [ ] Provide full pseudocode.
- [ ] Define objective terms and weights.
- [ ] Define convergence and termination criteria.
- [ ] Define gating conditions.
- [ ] Define replica/candidate counts.
- [ ] Define annealing schedule.
- [ ] Define rescue scopes and transitions.
- [ ] Define candidate selection.
- [ ] Define collision proxy and safety margin.
- [ ] State that one invocation may include multiple internal attempts.
- [ ] Provide computational-complexity/work discussion.
- [ ] Distinguish algorithm from implementation optimizations.

### 16.6 Experiments

- [ ] State hypotheses before outcomes.
- [ ] Describe development/held-out split.
- [ ] Describe serialized target construction.
- [ ] Describe all baseline implementations.
- [ ] Describe fairness and tuning protocol.
- [ ] Describe metrics and denominators.
- [ ] Describe statistical plan.
- [ ] Describe hardware and compiler environment.
- [ ] Describe collision engines and shared assumptions.
- [ ] Include release/config/target hashes.
- [ ] Declare confirmatory versus exploratory analyses.

### 16.7 Results

- [ ] Lead with the primary matched comparison.
- [ ] Present clean-success and conditional-collision results.
- [ ] Present ablations before speculative interpretation.
- [ ] Present latency and computational work together.
- [ ] Present held-out generalization.
- [ ] Present chain-length scaling with uncertainty and event counts.
- [ ] Present trajectory/hardware evidence if available.
- [ ] Present geometry/safety-margin sensitivity.
- [ ] Present null and negative findings.
- [ ] Keep weak textbook baselines as context, not the sole evidence.
- [ ] Avoid causal language not supported by ablation.

### 16.8 Discussion

- [ ] Label findings as supported, consistent, unsupported, or unknown.
- [ ] Explain how much gain comes from collision information.
- [ ] Explain how much gain comes from candidate selection.
- [ ] Explain how much gain comes from conditional gating.
- [ ] Explain how much gain comes from scoped rescue.
- [ ] Discuss conventional optimization interpretations.
- [ ] Discuss where ProteinIK loses.
- [ ] Avoid treating metaphor compatibility as mechanistic proof.

### 16.9 Limitations

- [ ] State simulation-only status if applicable.
- [ ] State self-collision versus environmental-collision scope.
- [ ] State proxy-geometry limitations.
- [ ] State internal ensemble cost.
- [ ] State possible trajectory discontinuity.
- [ ] State lack of guaranteed convergence.
- [ ] State lack of physical safety guarantee.
- [ ] State dependence on reachable-target construction.
- [ ] State tuning and held-out limits.
- [ ] State biological analogy limits.
- [ ] State baseline integration limitations.

### 16.10 Conclusion

- [ ] Lead with the measured technical contribution.
- [ ] State which components survived ablation.
- [ ] Bound conclusions to evaluated robots/scenarios.
- [ ] Describe folding as the design decomposition supported by evidence.
- [ ] Avoid new numbers or claims absent from Results.

**Gate 10:** the manuscript remains accurate if a skeptical reader ignores the biological metaphor entirely.

---

## 17. Rebuild figures and tables

### 17.1 Required figures

- [ ] Technical algorithm flow with biological inspiration in a separate lane.
- [ ] Pose success with paired confidence intervals.
- [ ] Clean success with paired confidence intervals.
- [ ] Collision conditional on success for each engine.
- [ ] Safety-margin sensitivity.
- [ ] Latency ECDF or distribution plot with p50/p95/p99/max.
- [ ] Work-versus-quality Pareto plot.
- [ ] Ablation forest plot with paired effects.
- [ ] Chain-length scaling with counts and intervals.
- [ ] Development-versus-held-out generalization plot.
- [ ] Failure taxonomy.
- [ ] Trajectory continuity/deadline plot if trajectory claims are made.

### 17.2 Required tables

- [ ] Robot, DOF, joint-limit, geometry, and collision-model details.
- [ ] Solver objective awareness, implementation, internal candidates, and budgets.
- [ ] Primary paired comparisons with counts, effects, and intervals.
- [ ] Complete ablation table.
- [ ] Latency and computational-work table.
- [ ] Reproducibility environment and artifact hashes.
- [ ] Negative and null results.
- [ ] Related-work capability comparison.

### 17.3 Figure/table QA

- [ ] Generate from approved trial-level data only.
- [ ] Drive solver inclusion from declared configuration.
- [ ] State sample size and denominator.
- [ ] State solver field.
- [ ] State scoring/collision engine.
- [ ] State uncertainty method.
- [ ] Use consistent units, colors, ordering, and naming.
- [ ] Make color choices accessible in grayscale/color-blind viewing.
- [ ] Avoid truncated axes that exaggerate differences.
- [ ] Avoid unstable ratios near zero.
- [ ] Inspect labels at final publication size.
- [ ] Cross-check every displayed value against generated tables.

---

## 18. Reproducibility package

- [ ] Provide a clean installation guide.
- [ ] Pin Python dependencies.
- [ ] Pin native dependencies.
- [ ] Provide container/WSL/environment instructions.
- [ ] Provide one command to build native solvers.
- [ ] Provide a fast smoke benchmark.
- [ ] Provide one command per paper experiment.
- [ ] Provide immutable target datasets.
- [ ] Provide trial-level results or a public archival location.
- [ ] Provide aggregation scripts.
- [ ] Provide figure/table generation scripts.
- [ ] Provide artifact checksums.
- [ ] Provide expected runtime, memory, CPU, and disk requirements.
- [ ] Document licenses for code, robot models, meshes, and dependencies.
- [ ] Provide CITATION metadata.
- [ ] Provide a mapping from each paper figure/table to its generation command.
- [ ] Run the reproduction on a clean machine or clean VM/container.
- [ ] Record deviations and fix undocumented assumptions.
- [ ] Have a second person reproduce the smoke benchmark.
- [ ] Archive the exact submission artifact with a persistent identifier if possible.

**Gate 11:** an independent person can regenerate the primary tables and figures without undocumented intervention.

---

## 19. Red-team review

For every use of “first,” “novel,” “best,” “fastest,” “cleanest,” “all,” “proves,” “real-time,” “independent,” or “significant”:

- [ ] Record the exact definition.
- [ ] Record the eligible comparison set.
- [ ] Record the source artifact.
- [ ] Record statistical support.
- [ ] Search for a counterexample.
- [ ] Weaken or delete the wording if any requirement fails.

Hostile-review questions:

- [ ] Would collision-aware multi-start reproduce the collision gain?
- [ ] Does the result survive removal of maximum-clearance selection?
- [ ] Is gating still useful after matching replica count and work?
- [ ] Does scoped rescue beat global restart on the same objective and budget?
- [ ] Does the biological framing predict anything beyond standard optimization theory?
- [ ] Were held-out robots genuinely untouched during tuning?
- [ ] Does the advantage persist under independent geometry and safety margins?
- [ ] Is the high-DOF result supported by enough successful events?
- [ ] Does the method generate continuous executable trajectories?
- [ ] Can every number be regenerated from one approved release?
- [ ] Are failures, crashes, and negative results fully included?
- [ ] Are comparisons fair when internal candidates and implementation languages differ?
- [ ] Is any figure excluding a solver that would weaken its caption claim?
- [ ] Is simulator agreement being overstated as physical safety?
- [ ] Would the contribution remain useful without the protein metaphor?

- [ ] Assign at least one reviewer who did not implement the method.
- [ ] Record every red-team criticism and resolution.
- [ ] Keep unresolved criticisms in the limitations or future-work section.
- [ ] Repeat claim-ledger review after all edits.

---

## 20. Final submission gate

### Scientific validity

- [ ] No designated result contradicts the prose.
- [ ] The central comparison includes a matched collision-aware baseline.
- [ ] Internal replicas and candidate selection are transparent.
- [ ] Gating, collision cost, annealing, rescue, and selection are separately tested.
- [ ] Primary comparisons are paired and uncertainty is reported.
- [ ] High-DOF claims have sufficient events or are explicitly inconclusive.
- [ ] Held-out evaluation supports the stated generalization scope.
- [ ] Negative results are retained.

### Benchmark integrity

- [ ] Final target/config/result artifacts have checksums.
- [ ] All final cells come from compatible commits and configurations.
- [ ] Resume provenance is complete.
- [ ] Tests and manuscript consistency checks pass.
- [ ] Figures and tables regenerate from immutable data.
- [ ] Hardware and timing environment is fully documented.

### Writing integrity

- [ ] Solver fields and exclusions are explicit everywhere.
- [ ] Denominators and sample sizes are stated.
- [ ] No zero-denominator ratio rhetoric remains.
- [ ] “Real-time” appears only if deadlines and end-to-end behavior justify it.
- [ ] Simulator cross-checking is not called physical validation.
- [ ] Folding terminology is clearly inspiration/analogy unless stronger evidence exists.
- [ ] Abstract and conclusion contain only completed claim-ledger items.
- [ ] Limitations match the final evidence.

### Reproducibility and release

- [ ] Clean-environment reproduction succeeds.
- [ ] Artifact package contains code, configs, targets, raw trials, analyses, and hashes.
- [ ] Licenses and citation metadata are complete.
- [ ] Submission PDF has been visually inspected page by page.
- [ ] References, equations, figures, and cross-references resolve.
- [ ] Anonymous-review requirements are satisfied where applicable.
- [ ] Final repository tag and archival package are created.
- [ ] Final red-team sign-off is recorded.
- [ ] Project owner approves submission.

**Final decision:**

- [ ] **READY** — all mandatory gates pass and every headline claim is traceable.
- [ ] **READY WITH DECLARED LIMITATIONS** — remaining gaps are nonfatal and explicitly bounded.
- [ ] **NOT READY** — one or more mandatory scientific, integrity, or reproducibility gates fail.

---

## 21. Reframing decision if the experiments disagree with the current story

- [ ] If all major components help, present the combined algorithmic/design contribution.
- [ ] If clearance selection explains most gains, reframe around candidate generation and selection.
- [ ] If the collision objective explains most gains, narrow the novelty claim accordingly.
- [ ] If gating improves speed but not quality, present compute allocation as the main contribution.
- [ ] If established collision-aware solvers win, publish an honest comparative/negative result if valuable.
- [ ] If no robust advantage remains, preserve the benchmark and translation case study without claiming solver superiority.
- [ ] Never discard valid negative outcomes solely because they weaken the original narrative.

The best final paper is the strongest claim that survives this checklist—not the strongest claim that can be written.
