# Archive — development history

> **Not authoritative.** Where anything here disagrees with
> [`paper/academic.md`](../../paper/academic.md) or the committed results in
> [`backend/results/`](../../backend/results), those win. This material is kept because
> a technical report / thesis will be written from it, and because it is the record of
> how the work actually went — including the paths that were tried and dropped.

| Path | Contents |
| :-- | :-- |
| [`drafts/`](drafts) | Earlier manuscript builds and their scaffolding — `paper_draft_v1.md`, `paper_final.md` (the July build, superseded), `paper_notes.md` (the claim→evidence spine), `outline_simple.md` (the plan), `simple.md` (plain-English mirror of the earlier build). |
| [`v5-cchik/`](v5-cchik) | The V5 line — Conflict-Controlled Homotopy IK: `protein_ik_v5_deep_research.md` (design, after three self-review passes) and `v5_research_report.md` (implementation + validation). The code still ships (`backend/app/solvers/protein_homotopy/`), but V5 is not in the current paper. |
| [`research_notes/`](research_notes) | A consolidated research pass over the codebase, docs and result files, every claim traced to `file:line`. Start at `00_INDEX.md`; `07_discrepancies.md` lists the contradictions that were found and resolved. |
| [`research_forks/`](research_forks) | Two falsifiable side-studies asking whether the V5/V6 machinery has value outside IK. |
| [`dev-log/`](dev-log) | `raw_notes.md` (the long running log), `sim_migration_plan.md` (how the dual-engine validation harness was built and validated, phase by phase), `research_direction.md` (the spectrum-of-solvers framing), `usecase_experiments.md` (the deployment-role studies). |
| [`upgrade-plan.md`](upgrade-plan.md) | A deferred plan for strengthening the work *after* the initial draft. It opens with an explicit "do not use yet" banner — a roadmap, not instructions. |

## Reading the archive: version numbers → paper names

| Archive | Code id | Paper |
| :-- | :-- | :-- |
| V1 / ProteinIK | `protein_ik` | StagedFold |
| V4 / ProteinIK Fast | `protein_fast` | KineticFold |
| V5 / CCH-IK | `protein_homotopy` | *(not in the paper)* |
| V6 / Raw Biology | `protein_raw` | LangevinFold (future work, §6) |

## Two things the archive gets wrong

Recorded here so they are not carried forward into the technical report:

1. **"Biophysics buys quality."** `drafts/paper_final.md` §3.4 read LangevinFold's low
   collision rate as evidence that the folding physics improves solution quality. It
   does not follow: the low collision comes from its multi-start structure and its
   clash-free filter, not from the Langevin dynamics. The current paper moves
   LangevinFold to future work and makes no such claim.
2. **Latency figures predating the native port.** Anything in the archive quoting
   ProteinIK at ~9–35 ms is measuring the *Python interpreter*, not the algorithm. The
   same logic compiled runs ~100–120× faster; the paper's timings are all from the
   native path (see [`../REPRODUCE.md`](../REPRODUCE.md) §B.0).
