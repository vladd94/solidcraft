# Findings: Per-Turn Helical Thread Geometry and Rendering

**Scope:** Diagnostic audit only. No production code changes.  
**Export commit:** `6a6ab6ec6b42f643fb1a24f160d8f0bc463286d7`  
**Files audited:**

- `tmp/per-turn-m6-export/per_turn_m6_plate.step`
- `tmp/per-turn-m6-export/per_turn_m6_plate.json`

---

## Executive summary

The exported solid is a closed, valid curved B-rep with exact **0.8 mm** pitch and a natural plane-intersection entrance. The concentric / poorly shaded viewport look is **not** primarily bad ISO math — it comes from **`tess::plan` rejecting a flank face**, then **`Body::tessellate` falling back to Truck’s mesh**, which produces a ring-like display mesh.

| Area | Verdict |
|------|---------|
| Authoritative CAD surfaces | Mostly correct (pitch exact; cylinders correct; truncated ISO profile) |
| Topological closure | OK (`validity` empty) |
| Geometric seam continuity | Minor ~10 µm G⁰ residual; normals agree |
| Natural entrance | OK (on plane; no bore chord; no radial end-caps) |
| Viewport appearance | **Broken by Truck fallback after plan CDT failure** |

---

## 1. Exact model (filename is misleading)

Despite `per_turn_m6_plate.*`, the design JSON specifies:

| Field | Value |
|-------|-------|
| Feature | `Hole1` |
| `thread` | `M5x.8` |
| `diameter` | `4.2 mm` (tap drill) |
| `depth` | `10 mm` |
| `modeled` | `true` |
| Hole axis | `+Y` (plate extruded from XZ) |

### STEP inventory

| Entity | Count |
|--------|------:|
| `MANIFOLD_SOLID_BREP` | 1 |
| `CLOSED_SHELL` | 1 |
| `ADVANCED_FACE` | 34 |
| `B_SPLINE_SURFACE` | 26 |
| `CYLINDRICAL_SURFACE` | 2 |
| Planar faces (classified after import) | 6 |
| Faceted / triangulated STEP faces | 0 |

**Reimport:** `faces=34`, `shells=1`, `validity` empty, not an open surface.  
**Breakdown:** Plane 6 + Cylinder 2 + BSpline 26 = 34.

---

## 2. Per-turn surface construction

### Why 26 B-spline faces?

Per-turn split in `internal_thread_wall_face_specs`:

- `n_seg = ceil(height / pitch) = ceil(10 / 0.8) = 13`
- Lower flank: 13 faces; upper flank: 13 faces → **26 B-spline flanks**
- Crest + root stay **analytic cylinders** (2), not B-splines
- Outer box + entry/exit planes: 6 planes

So 26 is exactly `2 × ceil(10/0.8)`. It is the current “≤1 revolution per flank face” strategy, **not** a polygonal / faceted helix.

Native wall FaceSpecs for the same M5×0.8×10 mm: **28** (= 2 revolved + 26 ruled) before sewing into a plate.

### Seam measurements (all 24 BSpline–BSpline shared edges)

| Metric | Measured |
|--------|----------|
| Shared mid-seams counted | 24 |
| Max positional deviation along seam | **9.984×10⁻³ mm** (~10 µm) |
| Worst oriented normal dot `n·n'` | **1.000000** |
| Worst projected-tangent `\|t·t'\|` | **1.000000** |

**Interpretation:**

- Topological sharing is present (manifold incidence 2 on each mid-seam).
- Oriented normals agree (no reversed neighbor pair on these seams).
- ~10 µm is a **geometric approximation residual** of the plane∩ / ruled B-spline construction — **not** proof of a missing edge.
- **Do not confuse topological closure with geometric or tangent continuity.** Validity empty ≠ zero-gap G⁰/G¹.

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

---

## 3. Natural thread entrance / exit

Entry/exit planes (import faces **32** and **33**, each 8 edges — outer rect + hole loop):

| Plane face | Shared BSpline edges | max off plane | max off parent surface | min radius on curves |
|----------:|---------------------:|--------------:|-----------------------:|---------------------:|
| 32 | 2 | 1.8×10⁻¹⁵ mm | 8.893×10⁻³ mm | **2.1000** |
| 33 | 2 | 8.9×10⁻¹⁵ mm | 8.893×10⁻³ mm | **2.1000** |

Verified:

- Entry trimming curves lie on the entry plane (machine epsilon).
- Curves also lie on parent helical surfaces within ~9 µm.
- `min_r = 2.1000` = crest → **no straight chord into the pilot bore**.
- No radial end-cap faces inside the groove.
- No evidence of an artificial depth ramp beyond the plane trim.

---

## 4. Viewport appearance — root cause

| Check | Result |
|-------|--------|
| `tess::plan` on exported solid | **FAIL** — `face 8: boundary crosses itself` (~1.5 s) |
| Face 8 type | **BSpline flank** (faces 5–30 are the 26 flanks) |
| `turns_more_than_once` | `true` → display path tries plan first |
| `Body::tessellate(2e-3)` | succeeds via **Truck `mesh_shells` fallback** (~3.9 s, 39743 tris) |
| L–R crest peaks on that mesh | left=10, right=10, **aligned≈9**, half-pitch≈0 → **ring signature** |
| B-rep helix edges | pitch exactly 0.8 |

### Hypothesis classification

| Hypothesis | Verdict |
|------------|---------|
| A. Incorrect mathematical surfaces | **Mostly no** — pitch exact; cylinders correct; profile matches truncated ISO |
| B. Incorrect trims / orientation | **Minor** — entry OK; seam G⁰ ~10 µm; normals agree |
| C. Discontinuous per-turn patches | **Minor G⁰ residual** (~10 µm); not the ring mechanism |
| D. Truck tessellation of trimmed B-splines | **YES — primary viewport defect** after plan CDT rejection |
| E. Viewport shading only | **Secondary** — it draws the fallback mesh |

Native M5 plate (same pitch/height family) also fails `tess::plan` (`face 10`), so this is not unique to STEP round-trip.

**Do not change the B-rep merely to make the viewport look better.** Viewport triangles are an acceptable display representation; mesh geometry must never replace the authoritative curved CAD surfaces. The fix belongs in making flank UV domains plan-compatible (or a display-only seam strategy).

---

## 5. Performance (single runs; no repeated validity)

| Step | Time |
|------|------|
| STEP import | ~241 ms |
| Wall FaceSpecs (M5×0.8×10) | ~0.7 ms |
| Native plate sew+stitch | ~10.6–10.7 s |
| `tess::plan` (fail) | ~1.4–1.5 s |
| `Body::tessellate` fallback | ~3.9 s |
| Validity | empty (one check only) |

Splitting into 26 flank faces adds FaceSpec/sew edge count versus 2 long flanks, but the dominant costs observed here are still sew/stitch (~10 s) and failed-plan + fallback tess (~5 s combined). Specs construction itself is sub-millisecond.

---

## 6. Guardrails observed

- No production code left modified (diagnostic harness was uncommitted and reverted).
- No thread-geometry redesign.
- No faceted geometry introduced.
- No geometry-validity checks weakened.
- No full-workspace builds.
- No four-hole integration work.

---

## Related files

| File | Purpose |
|------|---------|
| `per_turn_m6_plate.step` | Exported STEP solid |
| `per_turn_m6_plate.json` | Design JSON (M5×0.8) |
| `per_turn_m6_plate_step_json.zip` | Downloadable STEP+JSON |
| `audit_per_turn_m5_export.log` | Raw measurement log |
| `AUDIT_REPORT.md` | Earlier audit write-up (same findings) |
