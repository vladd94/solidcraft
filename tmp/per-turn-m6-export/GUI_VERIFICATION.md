# GUI verification: Face-8 shape-fallback fix (M5 × 0.8)

**Branch:** `cursor/meridional-thread-integrate-8929`  
**Fix commit (preserved):** `ef3d1e42`  
**Tip at verification:** `e9660cc6` (docs only after fix; curved B-rep unchanged)  
**Fixture:** `tmp/per-turn-m6-export/per_turn_m6_plate.{step,solidcraft,json}` @ `6a6ab6ec`  
(M5 × 0.8 single hole despite the `m6` filename; STEP bytes unchanged)

**Scope of this report:** live SolidCraft GUI only. No production geometry, CAD
accuracy, or thread-parameter changes. GPU and CPU/reference paths are judged
separately. Failure-budget / shape-fallback guardrails left as-is.

---

## Sessions

| Path | Launch | Port | PID | Prefs (at inspect) | Notes |
|------|--------|------|-----|--------------------|-------|
| GPU Fine + tess | `./target/release/solidcraft --control 8955` | 8955 | 430568 | `display_quality=fine`, `renderer=gpu` | Vulkan MSAA 4× |
| Software / Normal | `SOLIDCRAFT_SAFE_MODE=1 … --safe-mode --control 8956` | 8956 | 431251 | `display_quality=normal`, still `renderer=gpu` | Safe-mode still boots Vulkan; “software” here means reference/Truck mesh when LOD does not refine |

Control tokens: `~/.local/share/solidcraft/control/8955.token`, `8956.token`.  
Screenshots via `ui.screenshot`; LOD metrics via `ui.inspect` → `display_lod`.

---

## Answers to the six questions

### 1. Does the GPU viewport show a continuous helical thread (not concentric rings)?

**Yes — continuous helix on the GPU Fine / display-plan path.**

Decisive evidence is the side view with **half-pitch crest offset** on opposite
walls (≈ 0.4 mm for pitch 0.8), which concentric rings cannot produce.

| Artifact | Role |
|----------|------|
| `/opt/cursor/artifacts/gpu_side_helix.png` | Half-pitch offset (primary) |
| `/opt/cursor/artifacts/verify_gpu_side_helix.png` | Retake; LOD `refined_triangles=1 032 210` |
| `/opt/cursor/artifacts/gpu_overview.png` | Spiral into bore |
| `/opt/cursor/artifacts/gpu_section_profile.png` | Climbing section profile |
| `/opt/cursor/artifacts/gpu_oblique.png` | Oblique spiral |

Planner mesh signature (not GUI, but confirms what the viewport draws when
`display_plan` is used):

| File | Mesh | Signature |
|------|------|-----------|
| `m5_before_truck_unwrapped.png` | Truck reference | Near-horizontal bands (rings) |
| `m5_after_plan_unwrapped.png` | `display_plan` after `ef3d1e42` | Diagonal bands @ pitch 0.8 |

Caveat: lead angle ≈ 2.9°, so a pure orthogonal side glance can *look* like
parallel ridges even when helical. Use opposite-wall offset / section / oblique.

### 2. Do natural thread entrance and exit look geometrically correct?

**Yes on the GPU path.** Entrance/exit match plane∩helix (elliptical /
progressive flank start), not a rectangular groove cut or stacked washer
termination.

Evidence: `gpu_entry.png`, `gpu_entry_plane.png`, `gpu_exit.png`,
`gpu_exit_plane.png`, `gpu_oblique.png`, `gpu_overview.png`.

### 3. Missing, inverted, or incorrectly shaded flank faces?

**No defects observed on GPU Fine views.** Flanks are present, continuous, and
shaded consistently (no black inverted patches, no holes in the flank sheet).

Evidence: `gpu_side_helix.png`, `gpu_section_profile.png`, `gpu_section_side.png`,
`verify_gpu_oblique.png`.

### 4. Does software / Normal (reference-mesh) still produce the ring artifact?

**Yes — when the viewport is on the Truck reference mesh (`refined_triangles=0`).
That is separate from the GPU plan fix.**

| Observation | LOD | Visual |
|-------------|-----|--------|
| `cpu_far_reference.png` (earlier Normal far) | `planned=0` path | Ring-like / stacked washers |
| `verify_cpu_far_planned0.png` / `verify_cpu_far90.png` | `refined_triangles=0`, `planned≥1` but no refine | Coarse reference display (ring-prone) |
| `cpu_side_helix_retake.png`, `verify_cpu_side_close.png` | `refined_triangles≈170k` | Looks helical — **because LOD refined the display plan**, not because Truck was fixed |

**Do not treat a correct GPU display, or a close-up Normal view with refined LOD,
as proof that the reference mesh path is fixed.** Headless
`solidcraft-cli snapshot` / `ui.render` still draw `ModelBody::mesh()` (Truck)
and remain ring-like by design of that path.

Safe-mode caveat: the process still reports `renderer: "gpu"` (Vulkan). Isolation
is by mesh source (Truck reference vs `display_plan` refine), not by a separate
CPU rasterizer.

### 5. Tessellation time, responsiveness, display memory (~594 798-triangle model)

Measured on the same fixture after the shape-fallback fix
(`/opt/cursor/artifacts/gui_mesh_budgets.log` + live `ui.inspect`).

**Offline planner / body (kernel test budgets):**

| Metric | Value |
|--------|-------|
| Import STEP | 281 ms |
| `display_plan` @ sag 2e−3 | **1.44 s**, **594 798** tris, `PlanStats { planar: 6, structured: 2, delaunay: 26 }`, 96 edges |
| Truck reference mesh | **2.58 s**, **39 743** tris |
| `Body::tessellate` | **1.49 s**, **594 798** tris (= plan; no Truck fallback) |

**Live GPU Fine session (PID 430568):**

| Metric | Value |
|--------|-------|
| Process RSS | **≈ 1.26 GB** (1 291 068 KB) |
| Peak display LOD cache | **≈ 179 MB** (`cache_bytes=188 041 720`) |
| Cumulative `planning_ms` (LOD) | up to **≈ 12.5 s** after close zoom refinements |
| Refined triangles (close) | up to **1 032 210** (viewport LOD can exceed base plan) |
| Frame time after warm | **≈ 0.3–0.7 ms** typical; spike **≈ 12.6 ms** during heavy refine |
| Overview (early) | `planning_ms≈424`, `refined_triangles=0`, cache ≈ 11 MB |

**Live Normal / safe-mode session (PID 431251):**

| Metric | Value |
|--------|-------|
| Process RSS | **≈ 0.67–0.69 GB** |
| Close refine | ~170k–187k tris, `planning_ms≈0.9–1.8 s`, cache ≈ 13–28 MB |
| Far, no refine | `refined_triangles=0`, frame ≈ 0.3–10 ms |

Responsiveness: after planning completes, orbit/pan stays interactive
(sub-millisecond to low-ms frames). First close zoom that triggers refine is the
slow part (multi-second `planning_ms` on Fine).

### 6. Could the display plan produce fewer triangles at the same sag?

**Not by coarsening tolerance** (that changes sag). At fixed sag **2e−3**, the
current plan is dense mainly because anisotropic helical flanks go through the
Delaunay path with the **`shape: false` retry** after `BoundaryConstraintSplit`.

Same-sag reduction is plausible only via future display-only work, for example:

- Getting `shape: true` to succeed on those flanks (no boundary split), or
- Structured / ruled flank grids instead of full CDT, or
- Better UV sampling that needs fewer Steiner points while keeping shared
  boundary nodes.

Those are **proposals only**. Not implemented. Guardrails (typed
`TriangulateError`, single `shape: false` retry, refuse empty triangulation,
preserve shared boundaries) stay as in `ef3d1e42`.

Coarser-tol exploratory counts (different sag — **not** recommended for
production accuracy):

| tol | tris | plan time |
|-----|------|-----------|
| 5e−3 | 189 840 | 527 ms |
| 1e−2 | 158 374 | 490 ms |
| 2e−2 | 22 222 | 33 ms |

---

## Separation of failures (required)

| Path | Status after `ef3d1e42` |
|------|------------------------|
| GPU viewport drawing `display_plan` / LOD refine | **Fixed** — helix, natural entry/exit, flanks OK |
| Truck / reference mesh (`ModelBody::mesh`, `refined_triangles=0`, CLI snapshot) | **Still ring-like** — unchanged; out of scope of the face-8 CDT fix |
| Curved B-rep / ISO pitch / meridional entry | **Unchanged** — preserved |

---

## Guardrails preserved

- Typed `TriangulateError` (`InvalidInput` / `CrossingConstraint` /
  `BoundaryConstraintSplit`)
- One `shape: false` retry only on `BoundaryConstraintSplit`
- Empty triangulation refused
- No helix / ISO / trim / sew / hole-eval / tolerance changes
- Fixture STEP hash unchanged

Regression still in tree: `m5_export_plan_succeeds_with_shape_fallback`.

---

## Related docs

| File | Role |
|------|------|
| `FINDINGS.md` | Geometry audit |
| `FACE8_INVESTIGATION.md` | Root cause |
| `SHAPE_FALLBACK_RESULTS.md` | Post-fix planner validation |
| `WRITEUP.md` | Fix writeup |
| `GUI_VERIFICATION.md` | This GUI report |

---

## Recommendation before further work

1. Treat GPU Fine display of this model as **verified successful** for the
   face-8 planner fix.
2. Treat Truck / reference-mesh ring look as a **separate remaining issue**
   (mesh policy / LOD / Truck sampling), not as a regression of `ef3d1e42`.
3. Do not change production geometry or sag to chase triangle count until an
   explicit same-sag display-plan redesign is approved.
