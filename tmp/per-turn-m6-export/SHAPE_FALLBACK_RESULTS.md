# Results: display-only shape-refinement fallback

**Commit:** see `git log -1 --oneline` on `cursor/meridional-thread-integrate-8929`  
**Fixture:** `tmp/per-turn-m6-export/per_turn_m6_plate.step` @ `6a6ab6ec` (unchanged; hash `fd25cb21…`)

## Change

In `crates/kernel/src/tess/cdt.rs` and `mod.rs` only:

1. `cdt::triangulate` returns `Result<…, TriangulateError>` with
   `InvalidInput` | `CrossingConstraint` | `BoundaryConstraintSplit`.
2. `plan_face` Delaunay path tries `shape: true`, then **once** with
   `shape: false` only on `BoundaryConstraintSplit`.
3. Empty triangle lists are refused (`empty triangulation`).
4. Final errors use `TriangulateError::as_str()` (no longer always
   “boundary crosses itself”).

No helix / ISO / trim / sew / hole-eval changes.

## Validation (M5 × 0.8 STEP)

| Check | Result |
|-------|--------|
| `tess::plan` whole solid | **OK** — `PlanStats { planar: 6, structured: 2, delaunay: 26 }` |
| Face 8 + all flanks | 26 Delaunay faces planned, none empty |
| Triangle count | **594 798** |
| Crest edges pitch ≈ 0.8 | **26** helical, **2** flat; max \|pitch−0.8\| = 1.8e−15 |
| `Body::tessellate` == plan | **equal** triangle counts (no Truck `mesh_shells` fallback) |
| `Body::display_plan` | **OK** (GPU viewport path) |
| Shared edge records | `plan.edges.len() == plan.edge_faces.len()`; shared pairs > 20 |
| STEP fixture bytes | **unchanged** vs git |
| Re-export | still has `B_SPLINE_SURFACE`, no `FACETED_BREP` |

Regression: `m5_export_plan_succeeds_with_shape_fallback`  
CDT unit tests: 4/4 pass.

## Before / after visuals

Headless `solidcraft-cli snapshot` / `ui.render` always draw the **reference mesh**
(`ModelBody::mesh()` → Truck), not the display plan — so those PNGs stay
byte-identical before/after this fix and still look ring-like. That path was
not changed (out of scope).

Evidence of the planner fix is the unwrapped (θ, z) sampling of the two meshes:

| Image | Mesh | Signature |
|-------|------|-----------|
| `m5_before_truck_unwrapped.png` | Truck reference | Near-horizontal bands (rings) |
| `m5_after_plan_unwrapped.png` | `display_plan` after fix | Diagonal bands at pitch 0.8 (helix) |

## Remaining rendering notes

- GPU viewport with tessellation enabled draws `display_plan` → should show the
  helical plan mesh after this fix.
- Software/CPU Normal quality without `gpu.tess` still uses the reference mesh
  and can look ringy; that is separate from the face-8 CDT failure.
- `shape: false` may leave slightly lower triangle quality on anisotropic
  flanks; shared boundary nodes stay unsplit.

## Attempts

One implementation attempt; validated under the 90 s test budget.
