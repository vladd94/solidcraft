# Diagnosis: sequential modeled Hole1 → Hole2

**Branch tip:** `cursor/meridional-thread-integrate-8929`  
**Preserved:** `ef3d1e42` (display-plan shape fallback) and meridional curved B-rep  
**Scope:** read-only; two-hole minimal reproduction only; no geometry / GPU / thread-parameter changes  
**Budgets applied:** boolean ≤ 90 s, `try_replace` ≤ 90 s, one validity per closed candidate, two same-cause stop  

---

## Executive finding

On the **current tip**, the production path

`solid.hole { modeled: true }` → `solid.hole { modeled: true }`

**succeeds** for two M5×0.8 through-holes on a 60×60×10 plate (kernel and engine). Hole1’s helical B-spline faces and plane∩helix entry loops survive Hole2’s Boolean; Hole2’s `try_replace` closes with empty validity.

The historically reported failure (open **Regular** shell after cutting a new bore into an already-threaded body) is **not reproducible** with this two-hole minimal case anymore. Prior agent notes and a **stale** Phase B comment still describe that failure; those notes predate (or were not updated after) the bore-rim match fix and later stitch/orientation work.

Plain-holes-first then modeled `solid.thread` remains a valid, well-tested order (Phase 4B / Phase B), but it is no longer required for two sequential modeled Hole features on this tip.

---

## Production call chain (modeled Hole feature)

| Step | Location | Role |
|------|----------|------|
| 1 | `crates/doc/src/eval.rs` — `FeatureKind::Hole` | Build tap-drill tools → `apply_traced(..., Operation::Cut)` |
| 2 | same | If `modeled`, `cut_modeled_thread` → `kernel::try_replace_cylinder_with_helical_thread` |
| 3 | `crates/kernel/src/helical_thread.rs` — `try_replace_cylinder_with_helical_thread` | Find bore cylinders; `cap_faces_for_bore`; `remove_faces` (bore + top + bottom); `internal_thread_wall_face_specs` → `sew_open`; `replace_bore_wire_on_face` ×2; `stitch`; one `validity()` |
| 4 | `crates/kernel/src/helical_thread_trim.rs` — `replace_bore_wire_on_face` / `wire_matches_bore_rim` | Identify the target rim on a multi-wire planar cap (incl. 2-arc boolean rims) and swap in the helical entry loop while **keeping other wires** |

Critical property for Hole2: `try_replace` **drops the entire top and bottom planar faces** (which already carry Hole1’s entry wires), then rebuilds them from clones via `replace_bore_wire_on_face`. Hole1’s wall free edges must rematch the **preserved** Hole1 wires on those rebuilt caps. That rematch is the historical failure locus.

---

## Two-hole measurements (this tip)

Fixture: 60×60×10, M5×0.8 (`tap_radius` = 2.1), centers `(15,15)` then `(45,15)` unless noted.

### Path A — production order (boolean then replace per hole)

| Stage | Result |
|-------|--------|
| Hole1 boolean + `try_replace` | OK — 34 faces, Closed, free edges 0, 26 B-spline flanks, validity 0 (~5.5 s replace) |
| Hole2 boolean into threaded body | OK — 36 faces, Closed, free 0, **26 flanks retained**, Hole2 cylinder selectable, validity 0 (~1.7 s) |
| Hole2 `try_replace` | OK — 62 faces, 52 flanks, both bores empty, validity 0 (~11.2 s) |
| Hole1 entry samples across Hole2 boolean | bot/top hausdorff **0.0 mm**; top rim curve kinds `(line,arc,bsp)=(0,0,4)` unchanged |
| `tess::plan` after both | OK — ~1.2M tris, `PlanStats { planar: 6, structured: 4, delaunay: 52 }` |

Closer centers `(15,30)→(27,30)`: same success pattern.

Engine Session (`solid.hole` modeled twice, with `depth: 10` and through-all): both OK (34 → 62 faces, validity 0).

Transcript-shaped `hole → hole → thread → hole`: modeled third hole into a body with one helical wall also OK (64 faces, validity 0).

### Path B — plain then thread (control)

Both plain booleans then two `try_replace`: OK — 62 faces, matching Path A.

---

## Historical root cause (why it used to fail)

Documented by Phase B comments and prior agent runs (exact error):

```text
helical thread stitch did not yield a single closed solid
(shells=1, surface=true, validity=["shell 0 is Regular, not closed"])
```

Earlier blocker (fixed):

```text
could not identify the bore wire on the planar face
```

### Sequence that failed

1. Hole1 modeled → closed solid with meridional / per-turn helical walls and plane∩helix entry wires on the caps.  
2. Hole2 **Boolean Cut** into that solid — usually produced a closed solid and kept Hole1 flanks, but rewrote planar caps (new edge identities; often **2-arc** rims for the new cylinder).  
3. Hole2 **`try_replace`**:
   - `remove_faces` removes shared top/bottom (and Hole2 bore).
   - `replace_bore_wire_on_face` must find Hole2’s rim among multiple wires and leave Hole1’s entry wire intact.
   - Without `wire_matches_bore_rim`, step failed at wire ID (`975c65c9` fixed this).
   - Even after wire ID, stitch sometimes left an **open Regular** shell: Hole1 wall free edges did not rematch the rebuilt caps → `surface=true` / Regular, not closed.

### Why plain-first looked better

All Boolean cuts run on an all-analytic plate. Each `try_replace` only removes one bore + caps that still carry **circle** (or already-correct entry) wires for remaining holes, without a Boolean that rewrites caps while helical walls are already attached. That is why Phase 4B / Phase B drill-all-then-thread was adopted as the safe order.

Relevant commits:

| Commit | Note |
|--------|------|
| `975c65c9` | `wire_matches_bore_rim` — 2-arc boolean rim ID |
| `593dcd37` | Avoid `validity()` stall on open try_replace shells |
| `87217b52` | Phase B plain-first + validity budget (documents cut-then-thread as blocked) |
| `3bc98535` | Per-turn flank B-rep (current meridional wall topology) |

---

## Topology answer (Hole1 faces across Hole2 Boolean)

On this tip, for the two-hole case:

| Question | Answer |
|----------|--------|
| Helical B-spline faces altered/lost? | **No** — 26 flanks before and after Hole2 boolean |
| Unmatched boundary edges after boolean? | **No** — free-edge count 0, shell Closed |
| Entry/exit wires damaged? | **No** — 0 mm hausdorff on Hole1 rim samples; B-spline rim edges retained |
| Failure after Hole2 `try_replace`? | **Not observed** |

So the live failure is **not** “Boolean deletes Hole1 flanks.” The historical failure was **downstream stitch / cap rematch** inside the second `try_replace`, after shared caps were removed and rebuilt.

---

## Stale documentation

`phase_b_four_sequential_m5_modeled_through_holes` still says cut-then-thread-per-hole “leaves stitch unable to close” and is “recorded as blocked.” That comment is **out of date** relative to the two-hole results above. No four-hole re-run was performed (per request).

---

## Smallest corrective approaches (proposal only — not implemented)

**Do nothing for two-hole topology** if the product goal is “Hole1 modeled → Hole2 modeled works”: it already does on this tip. Recommended follow-up: replace the stale Phase B comment and add a **two-hole sequential modeled regression** (engine or kernel) that locks Path A.

If the goal is **hardening** against the historical Regular-shell class of failure (belt-and-suspenders, or N>2 unknown):

1. **Smallest eval-order change (preferred if hardening):** In `FeatureKind::Hole` when `modeled`, keep today’s per-feature Boolean, but optionally defer `cut_modeled_thread` until the end of a contiguous run of modeled hole features on the same body (boolean all first, then `try_replace` each). Matches the proven Path B order without changing thread geometry.  
2. **Narrower kernel hardening:** In `try_replace`, when caps already contain other helical entry wires, assert free-edge rematch of those wires against the open remnant before calling `validity()`; fail fast with a typed error instead of Regular-shell ambiguity.  
3. **Not recommended:** faceted boolean fallback, mesh threads, or changing pitch / lead-in / GPU tessellation.

Out of scope / preserved: `ef3d1e42` GPU display-plan path, curved B-rep, ISO profile, natural lead-in.

---

## Evidence artifacts

| File | Content |
|------|---------|
| `/opt/cursor/artifacts/sequential_hole2_diag.log` | Path A vs Path B kernel dump |
| `/opt/cursor/artifacts/sequential_hole2_engine.log` | Engine two modeled holes |
| `/opt/cursor/artifacts/sequential_hole2_entry.log` | Entry-loop hausdorff 0 across Hole2 boolean |
| `/opt/cursor/artifacts/sequential_hole2_close.log` | Closer spacing + `tess::plan` |
| `/opt/cursor/artifacts/sequential_hhh.log` | hole→hole→thread→hole |
| This file | Root-cause write-up |

Temporary diagnostic tests were **reverted** after measurement; production sources unchanged aside from this report.

---

## Approval request

Please choose next step:

1. **Accept “works for two holes”** — add a regression test + refresh stale Phase B comment only.  
2. **Harden eval order** — defer modeled `try_replace` until after consecutive modeled-hole Booleans (approach 1).  
3. **Investigate N>2** — separate budgeted run (still not a full four-hole product test unless you ask).  

No implementation until you approve.
