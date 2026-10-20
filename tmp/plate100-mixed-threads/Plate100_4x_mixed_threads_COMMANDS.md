# Plate100 — 4 modeled threaded holes (mixed pitches)

**Plate:** 100 × 100 × 10 mm  
**Holes (through-all, `modeled: true`):**

| # | Position (mm) | Thread |
|---|---------------|--------|
| 1 | (25, 25, 10) | M5×0.8 |
| 2 | (75, 25, 10) | M6×0.9 |
| 3 | (25, 75, 10) | M5×0.75 |
| 4 | (75, 75, 10) | M6×0.8 |

No application code was changed; files were produced with existing `solidcraft-cli`.

---

## Download paths

| File | Path |
|------|------|
| **STEP** | `/opt/cursor/artifacts/Plate100_4x_mixed_threads.step` |
| **JSON (recipe)** | `/opt/cursor/artifacts/Plate100_4x_mixed_threads.json` |
| Design (JSON `.solidcraft`) | `/opt/cursor/artifacts/Plate100_4x_mixed_threads.solidcraft` |

Local copies: `tmp/plate100-mixed-threads/` (same filenames).

STEP size: 2 609 298 bytes. Contains `B_SPLINE_SURFACE` (curved modeled threads).

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

From `--timings` (`Plate100_4x_mixed_threads_run.json`):

| index | command | thread | duration |
|------:|---------|--------|----------|
| 0 | `solid.box` | — | **0.968 ms** |
| 1 | `solid.hole` | M5×0.8 @ (25,25) | **4 494.912 ms** (~4.49 s) |
| 2 | `solid.hole` | M6×0.9 @ (75,25) | **9 983.045 ms** (~9.98 s) |
| 3 | `solid.hole` | M5×0.75 @ (25,75) | **19 108.044 ms** (~19.11 s) |
| 4 | `solid.hole` | M6×0.8 @ (75,75) | **22 224.028 ms** (~22.22 s) |

**Total:** **55 811.044 ms** (~55.81 s)

`replace` / stitch / validity (from process log, inside each modeled hole):

| hole | sew_open | stitch | validity |
|------|----------|--------|----------|
| M5×0.8 | ~21 ms | ~139 ms | ~4.30 s (0 issues) |
| M6×0.9 | ~14 ms | ~220 ms | ~7.56 s (0 issues) |
| M5×0.75 | ~16 ms | ~335 ms | ~11.57 s (0 issues) |
| M6×0.8 | ~15 ms | ~442 ms | ~15.01 s (0 issues) |
