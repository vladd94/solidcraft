# Writeup: sequential modeled Hole Path A — Option 1

**Branch:** `cursor/sequential-hole2-diagnosis-8929`  
**Tip commit:** `f2c6c48d91ad166f52ffedee186c6e35136586bd`  
**Preserved:** `ef3d1e42` (GPU display-plan shape fallback) and meridional curved B-rep  
**Scope:** regression tests + comment updates only — no eval-order, geometry, B-rep, or tessellation changes

---

## Decision

Approved **Option 1**: lock the working two-hole sequential modeled Hole workflow with permanent regressions, and clarify stale Phase B / Phase 4B comments. Historical diagnostics in this folder are retained; the Regular-shell cut-then-thread failure is **not reproducible** for two holes on the current tip.

---

## Production path (unchanged)

```text
solid.hole { modeled: true }
  → boolean Cut (tap drill)
  → cut_modeled_thread
  → try_replace_cylinder_with_helical_thread
       (remove_faces → sew_open wall → replace_bore_wire_on_face → stitch → 1× validity)
```

Path A = that order **per hole** (boolean into a body that may already have helical walls).  
Path B = drill all plain holes first, then modeled `solid.thread` / `try_replace` (still supported).

---

## Permanent regressions

| Test | Crate | What it locks |
|------|-------|----------------|
| `production_two_sequential_modeled_m5_holes_stay_curved` | engine | Real Hole tool: 60×60×10, through-all M5×0.8 at (15,15) then (45,15) |
| `two_sequential_modeled_m5_path_a_stays_closed_curved` | kernel | Same Path A at kernel level: Closed shell, free edges 0, plane∩helix entrances, prior flanks retained |
| `four_sequential_modeled_m5_path_a_timed` | kernel | Path A ×4 with per-hole boolean / replace timings; STEP on success |

After each two-hole operation the checks require:

- One closed, oriented B-rep solid (not an open shell)
- No free edges (kernel companion)
- Existing helical B-spline faces remain (flank count non-decreasing)
- Designation **M5**, pitch **0.8**, tap drill **4.2 mm**
- Natural entrance samples on both plate planes
- Compact face count + STEP contains `B_SPLINE_SURFACE`, not `FACETED_BREP`

Expensive `validity()` runs only inside each successful `try_replace` (≤90 s budget there). Post-checks are structural.

### Comment updates

- `phase_b_four_sequential_m5_modeled_through_holes` — Path A is not inherently blocked for two holes; historical Regular-shell notes kept for reference.
- `production_four_m5_modeled_holes_stay_curved` — plain-first remains supported; points at the two-hole Path A regression.

---

## Two-hole results

| Test | Result | Wall time (approx.) |
|------|--------|---------------------|
| Kernel Path A | **ok** | ~28 s |
| Engine Hole tool | **ok** | ~20 s |

Fixture: 60×60×10 mm plate, M5×0.8, centers `(15,15)` → `(45,15)`, through-all.

---

## Four-hole Path A timed results

Centers: `(15,15)`, `(45,15)`, `(15,45)`, `(45,45)`.  
Workflow: Path A (modeled Hole order), **not** plain-first.  
Outcome: **all four succeeded**; no geometry stop; no validity budget breach.

| hole | center | boolean_s | replace_s (sew/stitch/1×validity) | faces | flanks |
|------|--------|-----------|-------------------------------------|-------|--------|
| 0 | (15,15) | 0.011 | 5.375 | 34 | 26 |
| 1 | (45,15) | 1.764 | 11.215 | 62 | 52 |
| 2 | (15,45) | 3.282 | 16.970 | 90 | 78 |
| 3 | (45,45) | 4.875 | 22.330 | 118 | 104 |

Validity (inside replace), from `SOLIDCRAFT_THREAD_TIMING`:

| hole | validity_s | issues |
|------|------------|--------|
| 0 | ~5.11 | 0 |
| 1 | ~10.21 | 0 |
| 2 | ~15.18 | 0 |
| 3 | ~19.86 | 0 |

Final body: **118 faces**, **104** B-spline flanks, Closed, both bores empty at each completed hole.

### STEP export (last valid body)

| Location | Size |
|----------|------|
| `/opt/cursor/artifacts/path_a_four_m5_sequential.step` | 2 633 411 bytes |
| `tmp/sequential-hole2/path_a_four_m5_sequential.step` | same (local copy) |

Timings table: `tmp/sequential-hole2/path_a_four_m5_timings.md`.

---

## Related files in this folder

| File | Role |
|------|------|
| `SEQUENTIAL_HOLE2_ROOTCAUSE.md` | Pre-Option-1 diagnosis |
| `SEQUENTIAL_HOLE2_DIAGNOSIS.md` | Kernel Path A vs B dump |
| `SEQUENTIAL_HOLE2_ENGINE.md` | Engine two modeled holes |
| `SEQUENTIAL_HOLE2_ENTRY_PRESERVE.md` | Entry-loop hausdorff across Hole2 boolean |
| `path_a_four_m5_timings.md` | Four-hole timing table |
| `path_a_four_m5_sequential.step` | Final Path A solid |
| `WRITEUP.md` | This document |

---

## Commit

```text
f2c6c48d docs: record Path A four-hole sequential timing results
8d42668c test(thread): lock two sequential modeled M5 Hole Path A
```

Tip: **`f2c6c48d`**.
