# Prompt — generate `fig_pipeline`

Paste everything below the line into a **code-generating** model (Claude, GPT, Gemini).
Attach `fig_pipeline_CONTEXT.md` alongside it.

> Do **not** use a raster image generator (DALL·E, Midjourney, Imagen). They garble small
> technical text and cannot produce editable vector output. The deliverable is SVG, which
> opens in Illustrator / Inkscape / Figma and exports to the PDF the paper needs.

---

You are producing **Figure 1** for a robotics research paper submitted to an IROS/ICRA-tier
venue. Read the attached `CONTEXT.md` first — it explains what the algorithm does and what
the figure must communicate. Then build to the specification below, which is authoritative.

## Output contract

- **One self-contained SVG file.** No external fonts, no images, no scripts, no CSS files.
- `viewBox="0 0 1000 470"`, no hard-coded `width`/`height` attributes (so it scales).
- Opaque white background rectangle covering the full canvas.
- `font-family="Times New Roman, STIXGeneral, Georgia, serif"` declared once on the root
  `<svg>` element and inherited.
- Every text element positioned with explicit `x`/`y` and an explicit `text-anchor`.
- Output the complete SVG source in a single code block. Nothing else.

## Style rules

This is a print research paper, not a slide. **No** drop shadows, gradients, 3-D effects,
icons, clip art, emoji, or decorative flourishes. Thin strokes, generous whitespace, small
type, high information density.

**Palette** — Okabe–Ito, colour-blind safe. Fixed; do not substitute.

| Role | Hex |
|---|---|
| Phase A, fast path | `#009E73` |
| Phase B | `#CC79A7` |
| Decision gate | `#E69F00` |
| Terminal boxes, arrows, body text | `#333333` |
| Italic analog annotations | `#7A7A7A` |

Boxes: `rx="6"`, stroke 2 units in the band colour, fill = the same colour at 8 % opacity
(use `fill-opacity="0.08"`). The gate is a diamond `<polygon>`, same stroke weight, no fill.

Arrows: stroke `#333333`, width 1.8, with a filled triangular marker ~9 units long defined
once in `<defs>`. All connector corners are square — no curves except the one loop-back.

Meaning must never depend on colour alone; every element is also labelled and positioned.

**Type scale** (SVG user units, already converted from points):

| Element | Size | Weight |
|---|---|---|
| Box / band titles | 15.5 | bold |
| Gate title | 14 | bold |
| Body lines | 12 | normal |
| Edge labels, gate criterion | 11.5 | normal |
| Analog annotations | 11.5 | *italic*, `#7A7A7A` |
| Footnote | 11 | normal |

Line spacing 15 units. Each **analog annotation is the last line inside its own box**, so it
binds unambiguously to what it names.

## Geometry

Follow these coordinates. You may nudge a box's height or a label's position so text fits
comfortably, but **do not change the topology** — which box connects to which, and in which
direction, is the content of this figure.

| Element | Geometry |
|---|---|
| Input label `q₀, T_target` | text at (88, 100), `text-anchor="end"` |
| arrow → Phase A | (96, 95) → (124, 95) |
| **PHASE A** box | x 126, y 34, w 286, h 122 |
| arrow → gate | (412, 95) → (442, 95) |
| **GATE** diamond | polygon (444,95) (532,35) (620,95) (532,155) |
| `yes` edge | (532, 155) → (532, 198); label at (546, 178) |
| `no` edge | polyline (620,95) → (900,95) → (900,194); label at (760, 86) |
| **PHASE B** container | x 126, y 200, w 650, h 170 |
| container title | (142, 220), `text-anchor="start"` |
| container note | (760, 220), `text-anchor="end"` |
| B1 coarse collapse | x 142, y 258, w 164, h 92 |
| B2 Metropolis funnel | x 332, y 258, w 208, h 92 |
| B3 analytic rescue | x 566, y 258, w 194, h 92 |
| arrow B1 → B2 | (306, 296) → (332, 296) |
| arrow B2 → B3 | (540, 290) → (566, 290); label `stall` at (553, 281) |
| loop-back B3 → B2 | polyline (663,350) → (663,362) → (436,362) → (436,350) |
| arrow Phase B → SELECT | (776, 270) → (800, 270); label `converged` at (788, 261) |
| **SELECT** box | x 800, y 194, w 190, h 96 |
| arrow SELECT → STABILITY | (895, 290) → (895, 330) |
| **STABILITY GATE** box | x 800, y 330, w 190, h 100 |
| arrow → output | (895, 430) → (895, 444) |
| output label `q*` | (895, 460), `text-anchor="middle"`, bold |
| footnote, 2 lines | (126, 452) and (126, 464), `text-anchor="start"` |

Two things this geometry is designed to get right — preserve both:

- The `no` branch runs **above and to the right of** the Phase B container, never through it.
- Both the `no` branch and Phase B's exit arrive at **SELECT**. That merge is load-bearing:
  the selection and stability check are shared by both paths.

## Exact text

Reproduce verbatim. Every symbol below is a real Unicode glyph — `q₀` `q*` `‖Δp‖` `‖Δω‖`
`δ` `λ` `λ₀` `φᵢ` `T₀` `T_f` `rₜ` `𝒩` `σ` `≤` `≥` `∧` `−` `·` `←` `→` `★`. Do not substitute
ASCII. A wrong symbol is a factual error in a published paper.

**PHASE A** *(green)*
```
PHASE A — barrierless ensemble
max_replicas = 6  (one budget, shared with Phase B)
replica 0 ← q₀,  replicas 1–5 ← random
adaptive LM polish, ≤ 30 steps · Eq. 18
λ₀ = 0.08, self-tuning on accept / reject
stop at first converged replica with d(q) ≥ 0
kinetic partitioning — the barrierless fraction
```
(last line italic grey)

**GATE** *(orange)*
```
frustrated?
no converged replica
is clash-free
```
(line 1 bold at 14; lines 2–3 at 11.5)

**Edge labels**
```
no — 79% fast path
yes — 21% escalate
```
Set each as two lines: the word (`no` / `yes`) on the first, the share on the second at 11.5.

**PHASE B container** *(pink)*
- title: `PHASE B — the full staged fold`
- note (right-aligned): `frustrated targets only · stops on the first clean fold`

**B1** *(pink)*
```
coarse collapse
detuned DLS · Eq. 13
10·δ iterations,  δ ∈ [1, 3]
hydrophobic collapse
```

**B2** *(pink)*
```
Metropolis funnel  ★
150 iterations · Eqs. 19–20
single-joint proposals, rₜ = 0.5 · 0.985ᵗ
geometric cooling T₀ = 0.3 → T_f = 0.01
folding funnel
```

**B3** *(pink)*
```
analytic rescue  ★
stall: 10-window progress < 2e−4
joint i* = argmaxᵢ φᵢ · Eq. 21
scope ladder [n/6, n/2, 5n/6, n]
chaperone action (GroEL)
```

**SELECT** *(dark grey)*
```
SELECT
argmax d(q) over every
converged candidate,
both phases
```

**STABILITY GATE** *(dark grey)*
```
STABILITY GATE
5 jitters, σ ≈ 1 mm tip
reject if Eq. 7 error rises
by > 10 tolerances
on ≥ 4 of 5 trials
native-state stability
```

**Output**: `q*`

**Footnote** — two lines, left-aligned at (126, 452) and (126, 464), size 11, `#333333`
```
★ replaces StagedFold's greedy Eq. 15 and finite-difference Eq. 17.  Italic labels name the folding process each element ports.
Gate shares measured on UR5 + Franka, n = 1800.
```

## Before you answer, check

1. Every box's text fits **inside** its box with ≥ 8 units of padding on all sides. If a line
   would overflow, increase that box's height and shift what follows — never shrink the font
   below the scale above, and never let text cross a border.
2. No two text elements overlap. No arrow passes through a box or a label.
3. The `no` polyline clears the Phase B container entirely.
4. Both the `no` branch and the Phase B exit terminate on the SELECT box.
5. The rescue loop-back runs **below** B2 and B3 and stays inside the container.
6. Every arrow has a visible head at its destination end.
7. Every special glyph rendered as Unicode, not ASCII.
8. Nothing extends outside `0 0 1000 470`.

## After generating

1. Open the SVG and confirm it renders as intended — text fit is the most common failure.
2. Export **`fig_pipeline.pdf`** (vector, fonts embedded or converted to outlines) and
   **`fig_pipeline.png`** (300 dpi) into `paper/figures/`.
3. The paper embeds the PNG at `figures/fig_pipeline.png`; LaTeX will use the PDF.
