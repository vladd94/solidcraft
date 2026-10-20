# Investigation: `tess::plan` failure on face 8 (M5 × 0.8 export)

**Scope:** Display / tessellation-planner diagnosis only. No B-rep geometry changes.  
**Export:** commit `6a6ab6ec6b42f643fb1a24f160d8f0bc463286d7`  
**File:** `tmp/per-turn-m6-export/per_turn_m6_plate.step`  
**Budget:** two diagnostic attempts, ≤90 s each (both succeeded).

---

## Verdict

The reported `face 8: boundary crosses itself` is a **mislabeled tessellation-planner failure**, not a genuine 3D trim self-intersection and not a UV-seam / wrapping artifact.

| Question | Answer |
|----------|--------|
| Is the trim malformed in 3D? | **No** — min non-adjacent boundary-chord separation ≈ **0.062 mm** |
| Does the UV trim polygon self-cross? | **No** — **0** UV segment crossings (dense and plan-path sampling) |
| Is it a periodic UV seam wrap? | **No** — `EvalSurface::Nurbs`, `period = [None, None]`, domain `[0,1]×[0,1]` |
| What actually fails? | **Ruppert `shape` refinement** splits a boundary constraint; `cdt::triangulate` returns `None` |
| Why the message? | Any `triangulate` `None` is mapped to `"boundary crosses itself"` in `plan_face` |

**Authoritative curved B-rep, 0.8 mm pitch, ISO profile, meridional entrance, and STEP export should be left unchanged.**

---

## 1. Face 8 isolation

After STEP reimport (face indexing matches prior audit):

| Property | Value |
|----------|-------|
| Face index | **8** |
| Surface | B-spline → `EvalSurface::Nurbs` |
| Degree / ctrl | `[3, 1]` / `[67, 2]` (cubic × ruled) |
| Orientation | `true` |
| Wires / edges | 1 wire, **4** edges (quad flank patch) |
| UV domain | `u,v ∈ [0,1]`, non-periodic |
| Plan face size | `h ≈ 2.843`, `speed ≈ [19.59, 0.472]` (~**41:1** anisotropy) |
| Plan-path boundary nodes | **272** (shared edge sampling at `tol = 2e-3`) |
| Known intersection UVs | **0** (all projected) |

Logical trim sides (dense sample, Δ along each edge):

| Edge | Role (approx.) | Δu | Δv |
|-----:|----------------|---:|---:|
| 0 | near-iso-v crest/root side | +0.710 | ~0 |
| 1 | meridional / helical side | −0.231 | +1.000 |
| 2 | opposite near-iso-v side | −0.710 | ~0 |
| 3 | other helical side | +0.231 | −1.000 |

Artifacts: `face8_uv_poly.csv`, `face8_plan_uv.csv`, `face8_plan_path_audit.log`.

---

## 2. Boundary in 3D vs UV

### 3D

- Plan-path polyline: **no** non-adjacent chord pair closer than **6.17×10⁻² mm**.
- Not a bow-tie or crossed trim in model space.
- Edges lie on the Nurbs surface within the planner’s projection accept (`max(10·tol, 1e-6)`).

### UV

- UV bbox ≈ `u ∈ [0.0294, 0.9706]`, `v ∈ [0, 1]` (interior of the unit square).
- **0** proper UV segment intersections (non-adjacent).
- Closing / corner coincidence only at shared vertices (expected).

**Conclusion:** There is **no** exact edge-pair self-intersection responsible for a geometric cross. The failure is not “edge *i* crosses edge *j* at (t,s)”.

---

## 3. Classification

| Hypothesis | Evidence | Verdict |
|------------|----------|---------|
| Genuine 3D self-crossing trim | min sep 0.062 mm | **Reject** |
| UV seam / period unwrap artifact | non-periodic Nurbs | **Reject** |
| Projection / wrong UV branch | projections succeed; simple UV polygon | **Reject** as primary |
| Incorrect trim assembly in B-rep | 4-edge manifold flank; STEP validity empty | **Reject** |
| Planner parameterization stress | `speed` ~41:1; lattice seeds **0** | **Contributing** |
| Shape refinement splits constraints | see staged results below | **Primary cause** |

### Staged `cdt::triangulate` on the exact plan-path UV loop

| Stage | Result |
|-------|--------|
| No refine, no seeds | **OK** |
| Lattice seeds, no refine | **OK** (seeds = 0) |
| Seeds + sag refine, `shape: false` | **OK** |
| Seeds + sag refine, `shape: true` | **FAIL** (`None`) |
| Full `tess::plan` on solid | **FAIL** `face 8: boundary crosses itself` |

Code path (`crates/kernel/src/tess/cdt.rs`): after Ruppert `cdt.refine`, if `num_constraints() != constraints` the function returns `None` (“A split constraint would put a point on the boundary the neighbour does not have”).  
`plan_face` then maps **every** `None` to `"boundary crosses itself"` (`crates/kernel/src/tess/mod.rs`), which is incorrect for this case.

`shape` is enabled for all non-planar faces:

```text
shape: !matches!(s, EvalSurface::Plane { .. })
```

---

## 4. Is the surface parameterization suitable?

Partially.

- The trim is a simple region in UV — suitable for constrained Delaunay **without** aggressive angle shaping.
- Metric scaling `speed ≈ [19.6, 0.47]` shows the cubic helical direction is far “faster” in millimetres per parameter than the ruled direction. That anisotropy makes isotropic UV angle limits fight the boundary and encourages constraint splits.
- Empty lattice seeds mean almost all interior quality is asked of sag + Ruppert, increasing pressure on boundary constraints.

This is a **planner / display** limitation relative to the existing Nurbs parameterization, not proof that the B-rep flank is wrong.

---

## 5. Smallest supported fix (proposal only — not implemented)

### Recommended (display-only)

In `plan_face`’s Delaunay path (`crates/kernel/src/tess/mod.rs`), when

`cdt::triangulate(..., shape: true)` returns `None`,

**retry once** with the same seeds/sag budget and `shape: false`.

- Preserves shared boundary nodes (no split constraints).
- Keeps sag refinement for surface accuracy.
- Does not alter B-rep, pitch, profile, entrance, or STEP.
- Matches the staged evidence: `shape: false` already succeeds on face 8.

Optional clarity (same PR or follow-up): if both attempts fail, emit a distinct reason (e.g. `CDT constraint split` / `boundary not CDT-simple`) instead of always saying `boundary crosses itself`.

### Explicitly out of scope / rejected here

| Change | Why not |
|--------|---------|
| Rewrite helical / ruled geometry | Guardrail; B-rep is fine |
| Faceted Boolean thread | Forbidden |
| Weaken shell validity | Forbidden |
| Coarser `tol` only | Resolution-only workaround; hides planner bug |
| Allow constraint splits in CDT | Would break watertight shared edge nodes between faces |
| Reparameterize flanks for display | Larger than needed; try shape-fallback first |

### Stop condition (satisfied)

If the trim had been malformed in 3D, we would stop and report a geometry defect. **It is not.** Proceeding with a display-only planner fallback is appropriate.

---

## 6. Expected outcome after the proposed fix

- `tess::plan` accepts face 8 (and likely other anisotropic flank patches that fail the same way).
- `Body::tessellate` no longer falls back to Truck `mesh_shells` for this solid.
- Viewport should lose the concentric-ring artifact while STEP / curved faces stay identical.

---

## Guardrails observed

- No production code left modified (diagnostic harness reverted).
- No geometry rewrite, no faceted Boolean, no validity weakening.
- No four-hole work; no full-workspace CI.
- Two diagnostic attempts only; each under the 90 s execution budget.
