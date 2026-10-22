# Plate100 — 4 modeled threaded holes (mixed pitches)

**Plate:** 100 × 100 × 10 mm  
**Holes (through-all, `modeled: true`):**

| # | Position (mm) | Thread |
|---|---------------|--------|
| 1 | (25, 25, 10) | M5×0.8 |
| 2 | (75, 25, 10) | M6×0.9 |
| 3 | (25, 75, 10) | M5×0.75 |
| 4 | (75, 75, 10) | M6×0.8 |

**Branch:** `cursor/thread-on-0183-8929`  
**Tip commit:** `67b34594da43ef11703f66cdd72699f22717282f`  
**App version:** SolidCraft **0.19.0** (upstream Release sync)  
No application code was changed for this run; files were produced with existing `solidcraft-cli` on the modeled-thread Path A stack.

---

## Download paths (this branch)

| File | Path in repo |
|------|----------------|
| **STEP** | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads.step` |
| **JSON (recipe)** | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads.json` |
| Design (JSON `.solidcraft`) | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads.solidcraft` |
| Timings JSON | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads_run.json` |
| UI screenshot | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads_ui.png` |
| This report | `tmp/plate100-mixed-threads/Plate100_4x_mixed_threads_COMMANDS.md` |

On GitHub (branch `cursor/plate100-0190-download-8929`), open those paths or use raw download, e.g.:

`https://github.com/vladd94/solidcraft/raw/cursor/plate100-0190-download-8929/tmp/plate100-mixed-threads/Plate100_4x_mixed_threads.step`

STEP size: **2 607 020** bytes. Contains **104** `B_SPLINE_SURFACE` entities (curved modeled threads); no `FACETED_BREP`.

---

## Commands sent

Recipe file (`Plate100_4x_mixed_threads.json`):

```json
[
  {"command": "solid.box", "params": {"length": 100, "width": 100, "height": 10, "body_name": "Plate"}},
  {"command": "solid.hole", "params": {"position": [25, 25, 10], "thread": "M5x0.8", "modeled": true}},
  {"command": "solid.hole", "params": {"position": [75, 25, 10], "thread": "M6x0.9", "modeled": true}},
  {"command": "solid.hole", "params": {"position": [25, 75, 10], "thread": "M5x0.75", "modeled": true}},
  {"command": "solid.hole", "params": {"position": [75, 75, 10], "thread": "M6x0.8", "modeled": true}}
]
```

CLI invocation:

```bash
solidcraft-cli run Plate100_4x_mixed_threads.json \
  --out Plate100_4x_mixed_threads.step \
  --save Plate100_4x_mixed_threads.solidcraft \
  --timings --results --json-out Plate100_4x_mixed_threads_run.json
```

---

## Per-command durations

From `--timings` (`Plate100_4x_mixed_threads_run.json`, clean re-run on tip `67b34594`):

| index | command | thread | duration |
|------:|---------|--------|----------|
| 0 | `solid.box` | — | **3.730 ms** |
| 1 | `solid.hole` | M5×0.8 @ (25,25) | **5 377.614 ms** (~5.38 s) |
| 2 | `solid.hole` | M6×0.9 @ (75,25) | **12 987.384 ms** (~12.99 s) |
| 3 | `solid.hole` | M5×0.75 @ (25,75) | **22 844.863 ms** (~22.84 s) |
| 4 | `solid.hole` | M6×0.8 @ (75,75) | **29 200.810 ms** (~29.20 s) |

**Total:** **70 414.598 ms** (~70.41 s)

Feature-level `results` (engine feature clock, same run):

| feature | name | ms |
|--------:|------|---:|
| 1 | Box1 | 2.274 |
| 2 | Hole1 | 5 376.559 |
| 3 | Hole2 | 12 679.416 |
| 4 | Hole3 | 22 124.063 |
| 5 | Hole4 | 27 836.979 |

`replace` / stitch / validity (from process log, inside each modeled hole):

| hole | sew_open | stitch | validity |
|------|----------|--------|----------|
| M5×0.8 | ~24 ms | ~187 ms | ~5.12 s (0 issues) |
| M6×0.9 | ~21 ms | ~336 ms | ~10.10 s (0 issues) |
| M5×0.75 | ~26 ms | ~519 ms | ~15.26 s (0 issues) |
| M6×0.8 | ~23 ms | ~687 ms | ~20.20 s (0 issues) |

---

## Geometry / eval summary

`solidcraft-cli eval` on the saved design:

| Metric | Value |
|--------|-------|
| Kernel faces | **118** |
| Kernel edges | **348** |
| Kernel vertices | **232** |
| Shells | **1** |
| Volume | **101 017.68 mm³** |
| Display face_types (approx.) | plane 6, cylinder 8, cone 18 (tess path) |
| STEP `B_SPLINE_SURFACE` | **104** |
| STEP `FACETED_BREP` | **0** |

Expected face budget: 6 plate planes + 8 plain cylinder segments (crest/root stubs) + **104** helical flank B-spline faces ≈ **118**.

---

## Timing comparison (prior Plate100 run vs this 0.19.0 tip)

| hole | Prior (~55.8 s total) | This run (~70.4 s total) |
|------|----------------------:|-------------------------:|
| M5×0.8 | 4.49 s | 5.38 s |
| M6×0.9 | 9.98 s | 12.99 s |
| M5×0.75 | 19.11 s | 22.84 s |
| M6×0.8 | 22.22 s | 29.20 s |
| **Total** | **55.81 s** | **70.41 s** |

Cost still dominated by per-hole `validity()` after stitch; wall time grows roughly with accumulated helical faces (Path A sequential).

---

## UI

Screenshot in this folder: `Plate100_4x_mixed_threads_ui.png` (SolidCraft 0.19.0, iso view of the four mixed modeled holes).
