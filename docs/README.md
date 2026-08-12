# Documentation

| Document | What it is |
| :-- | :-- |
| [`REPRODUCE.md`](REPRODUCE.md) | **Start here to verify the paper.** Clean checkout → rebuilt figures/tables → re-run benchmarks, at three levels of cost. Includes the environment, the C++ build, and the known reproduction gaps. |
| [`METHODOLOGY.md`](METHODOLOGY.md) | The deep methods write-up that the paper's §3 condenses. Every subsection opens with a plain-English summary, then gives the precise algorithm, weights and tolerances. |
| [`design/`](design) | Per-solver design records — how each solver arrived at its final form, including the alternatives that were rejected and why. |
| [`archive/`](archive) | Development history, kept deliberately. **Not authoritative for the paper.** |

## Design records

| File | Solver | Contents |
| :-- | :-- | :-- |
| [`design/kineticfold-barrierless-first.md`](design/kineticfold-barrierless-first.md) | KineticFold (`protein_fast`) | The tail diagnosis, the barrierless-first ensemble, the allocation-light FK primitives, and the naive tail-edits that were measured and rejected. |
| [`design/langevinfold-design.md`](design/langevinfold-design.md) | LangevinFold (`protein_raw`) | Term-by-term design under the filter "every element must have no existing IK equivalent". |
| [`design/langevinfold-math.md`](design/langevinfold-math.md) | LangevinFold | The free energy, the Langevin integrator, the cooling schedule and the `T→0` endgame, in closed form. |
| [`design/langevinfold-audit.md`](design/langevinfold-audit.md) | LangevinFold | Faithfulness audit of each term against the biophysics literature, done *before* implementation. |
| [`design/langevinfold-result-draft.md`](design/langevinfold-result-draft.md) | LangevinFold | Drop-in result paragraph, pending the dedicated study the paper's §6 promises. |

## What's in the archive, and why it's kept

`docs/archive/` is the development record. It is preserved because a technical
report / thesis will be written from it — it is *not* a second source of truth, and
where it disagrees with [`paper/academic.md`](../paper/academic.md) or
[`backend/results/`](../backend/results), those win.

| Path | Contents |
| :-- | :-- |
| `archive/drafts/` | Earlier manuscript builds and their scaffolding: `paper_draft_v1.md`, `paper_final.md` (the July build, superseded by `paper/academic.md`), `paper_notes.md` (the claim→evidence spine used while drafting), `outline_simple.md` (the plan), `simple.md` (plain-English mirror of the earlier build). |
| `archive/v5-cchik/` | The V5 research line — Conflict-Controlled Homotopy IK. The code still ships (`backend/app/solvers/protein_homotopy/`, `fixed_lambda_ik.py`) and appears in the dashboard, but V5 is **not part of the current paper**. |
| `archive/research_notes/` | A consolidated internal research pass over the codebase and result files, with every claim traced to `file:line`. Indexed by `00_INDEX.md`; `07_discrepancies.md` is the list of contradictions found and resolved. |
| `archive/research_forks/` | Two falsifiable side-studies testing whether the V5/V6 machinery has value outside IK. |
| `archive/dev-log/` | `raw_notes.md` (the long running log), `sim_migration_plan.md` (how the PyBullet/MuJoCo validation harness was built and validated, phase by phase), `research_direction.md`, `usecase_experiments.md`. |
| `archive/upgrade-plan.md` | A deferred plan for strengthening the work *after* the initial draft. It opens with an explicit "do not use yet" banner; treat it as a roadmap, not as instructions. |

### Names changed between the archive and the paper

The archived material uses version numbers; the paper uses names. When reading the
archive, translate:

| Archive | Code id | Paper |
| :-- | :-- | :-- |
| V1 / ProteinIK | `protein_ik` | StagedFold |
| V4 / ProteinIK Fast | `protein_fast` | KineticFold |
| V5 / CCH-IK | `protein_homotopy` | *(not in the paper)* |
| V6 / Raw Biology | `protein_raw` | LangevinFold (future work) |

One correction worth carrying forward: the earlier draft
(`archive/drafts/paper_final.md` §3.4) presented LangevinFold's low collision rate as
evidence that "biophysics buys quality". That attribution does not hold — the low
collision comes from its multi-start structure and clash-free filter, not from the
folding physics. The current paper moves LangevinFold to future work (§6) and makes no
such claim.
