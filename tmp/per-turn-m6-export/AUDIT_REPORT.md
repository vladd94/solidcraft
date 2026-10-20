# Audit: per-turn helical thread export (commit 6a6ab6ec)

## 1. Exact model (not the filename)

Despite `per_turn_m6_plate.*`, the design JSON specifies:

| Field | Value |
|-------|-------|
| Feature | `Hole1` |
| `thread` | `M5x.8` |
| `diameter` | `4.2 mm` (tap drill) |
| `depth` | `10 mm` |
| `modeled` | `true` |
| Hole axis | `+Y` (plate extruded from XZ) |

STEP inventory (matches claim):

| Entity | Count |
|--------|------:|
| MANIFOLD_SOLID_BREP | 1 |
| CLOSED_SHELL | 1 |
| ADVANCED_FACE | 34 |
| B_SPLINE_SURFACE | 26 |
| CYLINDRICAL_SURFACE | 2 |
| PLANE | 7 in file text / **6** classified planar faces after import |
| Faceted / triangulated faces | 0 |

Reimport: `faces=34`, `shells=1`, `validity` empty, not an open surface.

Kernel face breakdown after import: **Plane 6 + Cylinder 2 + BSpline 26 = 34**.

## 2. Why 26 B-spline faces?

Per-turn split in `internal_thread_wall_face_specs`:

- `n_seg = ceil(height / pitch) = ceil(10 / 0.8) = 13`
- Lower flank: 13 faces; upper flank: 13 faces → **26 B-spline flanks**
- Crest + root stay **analytic cylinders** (2), not B-splines
- Outer box + entry/exit planes: 6 planes

So 26 is exactly `2 × ceil(10/0.8)`, not an arbitrary facet count. It is necessary for the current “≤1 revolution per flank face” tessellation strategy; it is **not** a polygonal approximation of the helix.

Native wall FaceSpecs for the same M5×0.8×10 mm: **28** (= 2 revolved + 26 ruled) before sewing into a plate.

### Seam measurements (all 24 BSpline–BSpline shared edges)

| Metric | Value |
|--------|------:|
| Shared mid-seams counted | 24 |
| Max positional deviation along seam | **9.984×10⁻³ mm** (~10 µm) |
| Worst oriented normal dot `n·n'` | **1.000000** |
| Worst projected-tangent `|t·t'|` | **1.000000** |

Interpretation:

- Topological sharing is present (manifold incidence 2 on each mid-seam).
- Oriented normals agree (no reversed neighbor pair on these seams).
- ~10 µm is a **geometric approximation residual** of the plane∩ / ruled B-spline construction (same order as entry `max_off_parent`), **not** proof of a missing edge. It is also **not** G¹ proof beyond normal agreement—tangent-plane agreement of normals is necessary for G¹ but the 10 µm G⁰ residual remains.

### Pitch

From 52 crest/root helix edges (BSpline–Cylinder):

| Metric | Value |
|--------|------:|
| Mean pitch estimate | **0.800000 mm** |
| Max \|pitch − 0.8\| | **2.1×10⁻¹⁵ mm** |

Pitch is exact for M5×0.8.

### ISO 60° profile (truncated)

| Quantity | mm |
|----------|---:|
| Crest cylinder radius | 2.100000 (tap) |
| Root cylinder radius | 2.500000 (major/2) |
| Radial flank span Δr | 0.400000 |
| ISO basic H | 0.692820 |
| ISO 5/8 H | 0.433013 |

Δr = 0.40 < 5/8H because the generator uses the truncated metric form (crest at tap radius, root at major/2), not sharp V-threads. Flank faces are the ruled ISO flanks between those cylinders.

## 3. Natural entrance / exit

Entry/exit planes (import faces **32** and **33**, each 8 edges — outer rect + hole loop):

| Plane face | Shared BSpline edges | max off plane | max off parent surface | min radius on curves |
|----------:|---------------------:|--------------:|-----------------------:|---------------------:|
| 32 | 2 | 1.8×10⁻¹⁵ mm | 8.893×10⁻³ mm | **2.1000** |
| 33 | 2 | 8.9×10⁻¹⁵ mm | 8.893×10⁻³ mm | **2.1000** |

- Curves lie on the entry plane (machine epsilon).
- Curves lie on parent helical surfaces within ~9 µm.
- `min_r = 2.1000` = crest → **no straight chord into the pilot bore**.
- No radial end-cap faces inside the groove (flanks meet the plate planes via plane∩ edges only).
- No evidence of an artificial depth ramp beyond the plane trim.

## 4. Viewport appearance — root cause

| Check | Result |
|-------|--------|
| `tess::plan` on exported solid | **FAIL** `face 8: boundary crosses itself` (~1.5 s) |
| Face 8 type | **BSpline flank** (faces 5–30 are the 26 flanks) |
| `turns_more_than_once` | true → display path tries plan first |
| `Body::tessellate(2e-3)` | succeeds via **Truck `mesh_shells` fallback** (~3.9 s, 39743 tris) |
| L–R crest peaks on that mesh | left=10, right=10, **aligned≈9**, half-pitch≈0 → **ring signature** |
| B-rep helix edges | pitch exactly 0.8 |

### Classification

| Hypothesis | Verdict |
|------------|---------|
| A. Incorrect mathematical surfaces | **Mostly no** — pitch exact; cylinders correct; profile matches truncated ISO |
| B. Incorrect trims / orientation | **Minor** — entry OK; seam G⁰ ~10 µm; normals agree |
| C. Discontinuous per-turn patches | **Minor G⁰ residual** (~10 µm); not the ring mechanism |
| D. Truck tessellation of trimmed B-splines | **YES — primary viewport defect** after plan CDT rejection |
| E. Viewport shading only | **Secondary** — it draws the fallback mesh |

Native M5 plate (same pitch/height family) also fails `tess::plan` (`face 10`), so this is not unique to STEP round-trip.

**Do not change the B-rep to hide this.** Fix belongs in making flank UV domains plan-compatible (or a display-only seam strategy), without replacing curved faces by facets.

## 5. Performance (single runs, no repeated validity)

| Step | Time |
|------|------|
| STEP import | ~241 ms |
| Wall FaceSpecs (M5×0.8×10) | ~0.7 ms |
| Native plate sew+stitch | ~10.6–10.7 s |
| `tess::plan` (fail) | ~1.4–1.5 s |
| `Body::tessellate` fallback | ~3.9 s |
| Validity | empty (one check only) |

Splitting into 26 flank faces adds FaceSpec/sew edge count versus 2 long flanks, but the dominant cost observed here is still sew/stitch (~10 s) and failed-plan+fallback tess (~5 s combined). Specs construction itself is sub-millisecond.

## 6. Guardrails observed

No production code left modified. Diagnostic harness was run uncommitted and reverted. No four-hole work. No faceting introduced.
