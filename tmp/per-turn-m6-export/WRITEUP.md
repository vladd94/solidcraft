# Writeup: Face-8 tessellation fix (display-only shape fallback)

**Branch:** `cursor/meridional-thread-integrate-8929`  
**Tip commit:** `3e93524f6f36af7daa1f08639c08b7c77f236831`  
**Fix commit:** `ef3d1e42`  
**Fixture:** `tmp/per-turn-m6-export/per_turn_m6_plate.step` @ `6a6ab6ec` (M5 × 0.8 despite filename; STEP bytes unchanged)

---

## Problem

`tess::plan` rejected B-spline flank **face 8** with:

```text
not supported yet: tessellation: face 8: boundary crosses itself
```

`Body::tessellate` then fell back to Truck `mesh_shells`, producing the concentric-ring viewport look. The authoritative curved B-rep (0.8 mm pitch, ISO profile, meridional entrance) was not the root cause.

## Investigation summary

Face 8’s trim is **simple in 3D and UV**:

| Check | Measured |
|-------|----------|
| 3D non-adjacent chord separation | ≈ 0.062 mm (not crossed) |
| UV segment crossings | 0 |
| Surface | Non-periodic Nurbs, domain `[0,1]²` |
| `triangulate` without shape | OK |
| `triangulate` with `shape: true` | FAIL |

Ruppert **shape** refinement split a protected boundary constraint. Any `triangulate` failure was mislabeled as “boundary crosses itself.”

Full diagnostic: `FACE8_INVESTIGATION.md`.

---

## Fix (implemented)

Scope: `crates/kernel/src/tess/cdt.rs` and `crates/kernel/src/tess/mod.rs` only.

1. **`TriangulateError`** — typed failures:
   - `InvalidInput`
   - `CrossingConstraint` (genuine self-cross / constraint conflict)
   - `BoundaryConstraintSplit` (refinement would split a shared boundary)

2. **Retry policy** — in the Delaunay path:
   - Attempt CDT with `shape: true` (existing behavior for curved faces).
   - If and only if the error is `BoundaryConstraintSplit`, retry **once** with `shape: false`.
   - Preserve original boundary sampling, sag tolerance, seeds, and shared-edge vertices.

3. **Safety**
   - Never accept an empty triangulation (`empty triangulation`).
   - If both attempts fail, report `TriangulateError::as_str()` (not a blanket “boundary crosses itself”).
   - Same fallback applied in the `plan_pieces` CDT path.

**Not changed:** helix generator, ISO profile, B-rep trimming, sewing, modeled-hole evaluation, validity checks, tolerances.

---

## Validation (M5 × 0.8 STEP)

Regression: `m5_export_plan_succeeds_with_shape_fallback`  
CDT unit tests: 4/4 pass.

| Check | Result |
|-------|--------|
| `tess::plan` (whole solid) | **OK** — `PlanStats { planar: 6, structured: 2, delaunay: 26 }` |
| All helical flank faces | 26 Delaunay faces; none empty |
| Triangle count | **594 798** |
| Crest edges @ pitch 0.8 | **26** helical, **2** flat; max \|pitch − 0.8\| ≈ 1.8×10⁻¹⁵ |
| `Body::tessellate` | Same triangle count as plan (**no Truck fallback**) |
| `Body::display_plan` | **OK** (GPU viewport path) |
| Shared edges | `plan.edges` aligned with `edge_faces`; shared pairs > 20 |
| STEP fixture | Hash unchanged (`fd25cb21…`) |
| Re-export | Still `B_SPLINE_SURFACE`; no `FACETED_BREP` |

---

## Before / after evidence

Headless `solidcraft-cli snapshot` and `ui.render` always draw the **reference mesh** (`ModelBody::mesh()` → Truck), not the display plan. Those CLI PNGs stay ring-like and were **not** the path this fix changes.

Planner evidence is unwrapped (θ, z) sampling of the two meshes:

| File | Mesh | Visual signature |
|------|------|------------------|
| `m5_before_truck_unwrapped.png` | Truck reference | Near-horizontal bands (rings) |
| `m5_after_plan_unwrapped.png` | `display_plan` after fix | Diagonal bands at 0.8 mm pitch (helix) |

Detailed results: `SHAPE_FALLBACK_RESULTS.md`.

---

## Remaining notes

- GPU viewport with tessellation enabled uses `display_plan` → should show the helical plan mesh after this fix.
- Software/CPU Normal quality without `gpu.tess` still uses the reference mesh and can look ringy; that is a separate path.
- `shape: false` may leave slightly lower triangle quality on anisotropic flanks; shared boundary nodes stay unsplit.

Live SolidCraft GUI confirmation (GPU Fine vs Normal/reference, half-pitch helix,
entrance/exit, flanks, perf, triangle-budget notes): see `GUI_VERIFICATION.md`.

---

## Related files

| File | Role |
|------|------|
| `FINDINGS.md` | Geometry audit of the M5 export |
| `FACE8_INVESTIGATION.md` | Face-8 root-cause investigation |
| `SHAPE_FALLBACK_RESULTS.md` | Post-fix validation notes |
| `WRITEUP.md` | This document |
| `per_turn_m6_plate.step` / `.json` | Fixture (M5 × 0.8 modeled) |
