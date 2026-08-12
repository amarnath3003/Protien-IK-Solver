# The paper

| File | What it is |
| :-- | :-- |
| **[`academic.md`](academic.md)** | **The canonical draft.** IEEE-style: Abstract → Introduction → Related Work → Methodology → Experimental Setup → Results and Discussion → Conclusion and Future Work → References. All numbers are locked to the committed result files. |
| [`academic_simple.md`](academic_simple.md) | A plain-English mirror. Every sentence in `academic.md` has a simple version here, in the same order under the same headings; equations are explained in words instead of symbols, but nothing is dropped. |
| [`figures/`](figures) | Every figure plus the script that generates it from the committed benchmark files. Nothing is hand-transcribed — see [`figures/README.md`](figures/README.md). |
| [`tables/`](tables) | The generated LaTeX tables (`tab_*.tex`, from `figures/make_tables.py`) and the hand-authored static ones (`tables_static.tex`: the isomorphism, robots, scenario thresholds, baseline hyperparameters). |

Earlier manuscript builds are in [`../docs/archive/drafts/`](../docs/archive/drafts) —
they are development history, not a second source of truth.

## Structure of the argument

1. **The correspondence** (§1, §3.1) — IK and folding share their variables, their
   constraints, and the shape of the space they search. Stated formally as one search
   object every solver in the study operates on.
2. **The direction of the crossing** (§2) — prior transfers between the fields run
   robotics → biology and carry one *move* at a time. This one runs the other way and
   carries the *process*.
3. **Two solvers of increasing literalness** (§3.2, §3.3) — StagedFold ports folding's
   ordered sequence; KineticFold adds kinetic partitioning as a compute schedule. Every
   numerical ingredient is standard IK, so any advantage is the sequencing's.
4. **The evidence** (§5.1–§5.3) — success, latency and real-mesh self-collision against
   a field spanning the IK literature, on three arms × three scenarios.
5. **The test of the thesis** (§5.4) — lengthen a planar arm toward a polymer and the
   advantage grows where the problem becomes most folding-like.
6. **The validation** (§4.6, §5.6) — "solve once, score three ways", which confirms the
   success claims on two independent engines and *corrects* our own
   collision-magnitude claim downward.

## Figures and tables

| # | Item | Source |
| :-- | :-- | :-- |
| Table 1 | The folding / IK isomorphism | `tables/tables_static.tex` (hand-authored) |
| Figure 1 | KineticFold's compute schedule | `figures/fig_pipeline.{svg,pdf,png}` (hand-authored vector art) |
| Tables 2–4 | Robots · scenario thresholds · baseline hyperparameters | `tables/tables_static.tex` |
| Figure 2 | Success rate across the field, by arm | `figures/fig_success.py` ← 3-seed survey |
| Figure 3 | Per-solve latency (median, mean, p99) | `figures/fig_latency.py` ← 3-seed survey |
| Figure 4 | Real-mesh self-collision by scenario | `figures/fig_collision.py` ← 10-seed sweep |
| Table 5 | Clean-solve % vs. DOF (planar 4→16) | `figures/make_tables.py` ← DOF-scaling study |

## Rebuilding

```bash
cd figures
python build_all.py                 # every data-driven figure + all LaTeX tables
python build_all.py --with-solvers  # + the two figures that run the solvers
```

Full recipe, including how to regenerate the underlying benchmark data, is in
[`../docs/REPRODUCE.md`](../docs/REPRODUCE.md). Which result file backs which claim is
in [`../backend/results/README.md`](../backend/results/README.md).

## Converting to LaTeX

`academic.md` is the working master; it converts to a two-column LaTeX manuscript once
the sections are settled. The generated tables need `booktabs` (and the static ones
`makecell` / `array`); figures are vector PDF, included at `\columnwidth`.

## Solver naming

The code predates the paper's names. In the manuscript:

| Code id | Paper name |
| :-- | :-- |
| `protein_ik` | StagedFold |
| `protein_fast` | KineticFold |
| `protein_raw` | LangevinFold (future work, §6) |

The mapping is applied automatically in every figure and table by
`figures/_style.py`, so a benchmark CSV never has to be renamed by hand.
