# LangevinFold — mini result paragraph (drop-in for §5)

Run first (on the sim env), then fill the three PyBullet numbers from `tab:langevin`:

```
cd backend
PYTHONPATH=. .venv-sim/Scripts/python -m bench.langevin_benchmark   # -> results/langevin_bench.{csv,md}
```

The benchmark reuses the master harness (solve once, score three ways), so its numbers
are apples-to-apples with Tables 5–8. Default scale n = 120/cell (trials 40 × seeds
1–3) — small by design, because LangevinFold costs seconds per solve.

---

## Draft paragraph (numbers in ⟨…⟩ come from `tab:langevin` / `fig_langevin`)

**5.x  LangevinFold: faithful biophysics buys quality, not speed.**
LangevinFold (§3.4) is not a practical solver — its coarse-grained overdamped-Langevin
fold costs ≈⟨2.2⟩ s per UR5 solve (mean; p99 ≈⟨2.3⟩ s), two to three orders of magnitude
slower than every numerical baseline — so we benchmark it separately and at smaller
scale (n = ⟨120⟩ per cell), scored by the same two real-mesh oracles as the master sweep
(Table `tab:langevin`, Fig. `fig_langevin`). What that latency buys is solution
*quality*: on the non-redundant UR5, LangevinFold produces the lowest real-mesh
self-collision rate of any solver in the study, in every scenario and on both
independent engines (PyBullet ⟨12⟩ / ⟨31⟩ / ⟨51⟩ % on open / near-singular / cluttered,
versus ⟨29⟩ / ⟨40⟩ / ⟨56⟩ % for the next-cleanest solver, KineticFold; MuJoCo agrees to
within the ⟨1–2⟩ pp engine gap), while retaining ⟨100⟩ % task success. This is the
expected far end of the fidelity spectrum: StagedFold ports folding's *sequence* and
KineticFold its *schedule*, but LangevinFold runs the folding *physics* itself, and its
attractive Lennard-Jones core-packing term plus hard clash-free endgame selection (§3.4)
yield folds a schedule-level method does not. LangevinFold is therefore not a competitor
to KineticFold but a complement — an offline, quality-critical goal generator for cases
where a clean fold matters more than milliseconds — and a further sign that the
folding/IK correspondence keeps paying off the more literally it is taken.

---

## Notes on the ⟨placeholders⟩

- **Latency (≈2.2 s / p99 ≈2.3 s, 100 % success)** — already measured on UR5 in a smoke
  run; the full run will confirm/refine.
- **PyBullet collision ⟨12/31/51⟩ %** — these are the values from the paper's earlier
  `sim_crosscheck` (n=100, single seed), where LangevinFold was the cleanest UR5 solver
  in all three scenarios. The new `langevin_bench` (n=120, seeds 1–3, both engines)
  regenerates them; paste the `tab:langevin` PB row for LangevinFold and the
  next-cleanest solver.
- If LangevinFold is *not* cleanest in some cell on the fresh run, soften "in every
  scenario" to "in the harder scenarios" — the honest framing the paper already uses
  elsewhere.
