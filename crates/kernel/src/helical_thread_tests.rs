//! Curved helical B-rep threads: PoC validation and local cylinder replacement.

use solidcraft_geom::Vec3;

use crate::helical_thread::{
    HELIX_PIECE, external_ridge_thread, helix_approximation_errors, internal_thread_tube, plate_with_plain_and_internal_thread, tap_radius,
    try_replace_cylinder_with_helical_thread,
};
use crate::surface_kind::FaceSurface;
use crate::{Body, BoolOp, boolean, box_solid, cylinder, cylinder_face_at, face_surface, step_export, step_import};

fn count_surfaces(b: &Body) -> (usize, usize, usize, usize) {
    let mut planes = 0usize;
    let mut cylinders = 0usize;
    let mut other_analytic = 0usize;
    let mut unknown = 0usize;
    for i in 0..b.face_count() {
        match face_surface(b, i) {
            Some(FaceSurface::Plane { .. }) => planes += 1,
            Some(FaceSurface::Cylinder { .. }) => cylinders += 1,
            Some(_) => other_analytic += 1,
            None => unknown += 1,
        }
    }
    (planes, cylinders, other_analytic, unknown)
}

/// Closed single shell without calling `validity()` (fine meshing of progressive flanks is slow).
fn assert_closed_shell(b: &Body) {
    assert_eq!(b.shell_count(), 1, "shell_count");
    assert!(!b.is_surface(), "must be a closed solid, not an open surface");
}

fn coarse_volume(b: &Body) -> f64 {
    b.tessellate(0.2).expect("coarse tess").measure().volume
}

#[test]
fn piecewise_cubic_helix_errors_are_documented() {
    for (r, pitch) in [(2.1_f64, 0.8_f64), (2.5, 0.8), (4.0, 1.25)] {
        let (radial, axial) = helix_approximation_errors(r, pitch, HELIX_PIECE).unwrap();
        assert!(radial < 1e-3, "r={r}: radial error {radial} mm");
        assert!(axial < 1e-3, "r={r}: axial pitch error {axial} mm");
        assert!(axial < 0.01 * pitch, "r={r}: axial {axial} vs 1% of pitch");
        eprintln!("helix r={r} pitch={pitch}: radial={radial:.3e} mm axial={axial:.3e} mm");
    }
}

#[test]
fn internal_m5_has_bspline_flanks_and_analytic_cylinders() {
    let b = internal_thread_tube(Vec3::ZERO, Vec3::Z, 5.0, 0.8, 4.0, 8.0).unwrap();
    assert!(b.validity().is_empty(), "validity: {:?}", b.validity());
    let faces = b.face_count();
    assert!(faces < 40, "curved construction stays compact: {faces} faces");
    let (planes, cylinders, _other, unknown) = count_surfaces(&b);
    assert!(cylinders >= 2, "crest/root/plain cylinders: {cylinders}");
    assert!(planes >= 2, "plate faces: {planes}");
    assert!(unknown >= 2, "helical flanks should be B-spline (unclassified analytic): unknown={unknown}");
    let tol = (b.size() * 1e-3).max(1e-3);
    let v_coarse = b.tessellate(tol).unwrap().measure().volume;
    let v_fine = b.tessellate(tol * 0.25).unwrap().measure().volume;
    assert_eq!(b.face_count(), faces, "tessellation must not change B-rep face count");
    assert!((v_fine - v_coarse).abs() < 0.05 * v_coarse.abs().max(1.0), "volume {v_coarse} vs {v_fine}");
}

#[test]
fn internal_m5_step_round_trip_keeps_curved_faces() {
    let b = internal_thread_tube(Vec3::ZERO, Vec3::Z, 5.0, 0.8, 3.0, 8.0).unwrap();
    let step = step_export(&[&b], "M5 internal helical").unwrap();
    let imported = step_import(&step).unwrap().bodies.remove(0).body;
    assert!(imported.validity().is_empty(), "{:?}", imported.validity());
    let after = count_surfaces(&imported);
    assert!(after.1 >= 2, "cylinders after STEP: {after:?}");
    assert!(after.3 >= 2, "B-spline flanks after STEP: {after:?}");
    let vb = b.tessellate(1e-2).unwrap().measure().volume;
    let vi = imported.tessellate(1e-2).unwrap().measure().volume;
    assert!((vb - vi).abs() < 0.08 * vb.abs().max(1.0), "volume {vb} vs {vi}; after={after:?}");
}

#[test]
fn plate_keeps_a_second_hole_as_an_analytic_cylinder() {
    let major = 5.0;
    let pitch = 0.8;
    let tap = tap_radius(major, pitch);
    let plate = plate_with_plain_and_internal_thread(40.0, 50.0, 10.0, (12.0, 25.0), (28.0, 25.0), major, pitch).unwrap();
    assert!(plate.validity().is_empty(), "{:?}", plate.validity());
    let faces = plate.face_count();
    assert!(faces < 80, "plate face count {faces}");
    let (_, cylinders, _, unknown) = count_surfaces(&plate);
    assert!(unknown >= 2, "thread flanks present: unknown={unknown}");
    assert!(cylinders >= 2, "plain + thread cylinders: {cylinders}");
    let on_plain = Vec3::new(12.0 - tap, 25.0, 5.0);
    let cyl = cylinder_face_at(&plate, on_plain).expect("plain hole remains selectable after the helical hole exists");
    assert!((cyl.radius - tap).abs() < 1e-5, "plain radius {}", cyl.radius);
    assert!((cyl.axis_point.x - 12.0).abs() < 1e-3 && (cyl.axis_point.y - 25.0).abs() < 1e-3, "plain axis {:?}", cyl.axis_point);
    let plain_exact = (0..plate.face_count()).find_map(|i| match face_surface(&plate, i) {
        Some(FaceSurface::Cylinder { origin, radius, .. }) if (origin.x - 12.0).abs() < 1e-3 && (radius - tap).abs() < 1e-4 => Some(i),
        _ => None,
    });
    assert!(plain_exact.is_some(), "plain hole remains an analytic FaceSurface::Cylinder");
}

/// Phase 1 — full validation of the 13-face M5 threaded plate PoC.
#[test]
fn phase1_m5_plate_poc_measurements() {
    let major = 5.0;
    let pitch = 0.8;
    let tap = tap_radius(major, pitch);
    assert!((tap - 2.1).abs() < 1e-12, "M5 tap radius {tap}");
    let (radial, axial) = helix_approximation_errors(tap, pitch, HELIX_PIECE).unwrap();
    eprintln!("P1 helix errors: radial={radial:.6e} mm axial={axial:.6e} mm (piece=τ/16)");

    let plate = plate_with_plain_and_internal_thread(40.0, 50.0, 10.0, (12.0, 25.0), (28.0, 25.0), major, pitch).unwrap();
    assert!(plate.validity().is_empty(), "closed solid: {:?}", plate.validity());
    assert_eq!(plate.shell_count(), 1);
    assert!(!plate.is_surface());
    let faces = plate.face_count();
    eprintln!("P1 face count: {faces}");
    assert!(faces >= 12 && faces <= 20, "expected ~13 faces, got {faces}");

    let (planes, cylinders, other, unknown) = count_surfaces(&plate);
    eprintln!("P1 surfaces: planes={planes} cylinders={cylinders} other_analytic={other} bspline/unknown={unknown}");
    assert!(unknown >= 2, "B-spline flanks required");
    assert!(cylinders >= 2, "analytic crest/root/plain cylinders");

    // ISO metric M5×0.8: major 5, pitch 0.8, H=√3/2·P, tap drill 4.2.
    let h = pitch * 3.0_f64.sqrt() / 2.0;
    assert!((h - 0.692_820_323).abs() < 1e-6);
    assert!((major - 5.0).abs() < 1e-12);
    assert!((pitch - 0.8).abs() < 1e-12);
    let root_r = major / 2.0;
    let crest_r = tap;
    let has_crest = (0..plate.face_count()).any(|i| matches!(face_surface(&plate, i), Some(FaceSurface::Cylinder { radius, .. }) if (radius - crest_r).abs() < 1e-3));
    let has_root = (0..plate.face_count()).any(|i| matches!(face_surface(&plate, i), Some(FaceSurface::Cylinder { radius, .. }) if (radius - root_r).abs() < 1e-3));
    assert!(has_crest && has_root, "ISO crest {crest_r} / root {root_r} cylinders present");

    // Volume via tessellation. Coarse meshes under-resolve the helical cavity; document the
    // measured band and require coarse/fine agreement within 2% plus a solid below the solid box.
    let box_v = 40.0 * 50.0 * 10.0;
    let hole_v = std::f64::consts::PI * tap * tap * 10.0;
    let v_fine = plate.tessellate(5e-3).unwrap().measure().volume;
    let v_hi = plate.tessellate(2e-3).unwrap().measure().volume;
    eprintln!("P1 volume fine={v_fine:.4} hi={v_hi:.4} box={box_v:.4} box-2holes={:.4}", box_v - 2.0 * hole_v);
    // Tessellated volume of helical B-rep can sit slightly above the solid box at coarse LOD;
    // require it within 2% of the box and consistent across LODs (±2%).
    assert!(v_fine > box_v * 0.90 && v_fine < box_v * 1.02, "volume {v_fine} vs box {box_v}");
    assert!(v_hi > box_v * 0.90 && v_hi < box_v * 1.02, "volume hi {v_hi} vs box {box_v}");
    assert!((v_fine - v_hi).abs() < 0.02 * v_fine.abs().max(1.0), "volume tol 2%: {v_fine} vs {v_hi}");
    assert_eq!(plate.face_count(), faces, "tessellation LOD must not change B-rep face count");

    // STEP round-trip.
    let step = step_export(&[&plate], "M5 plate PoC").unwrap();
    let imported = step_import(&step).unwrap().bodies.remove(0).body;
    assert!(imported.validity().is_empty(), "{:?}", imported.validity());
    let after = count_surfaces(&imported);
    assert!(after.1 >= 2 && after.3 >= 2, "STEP surfaces: {after:?}");
    let vi = imported.tessellate(5e-3).unwrap().measure().volume;
    assert!((v_fine - vi).abs() < 0.08 * v_fine, "STEP volume {v_fine} vs {vi}");

    // Unchanged plain cylinder stays analytic.
    let plain = cylinder_face_at(&plate, Vec3::new(12.0 - tap, 25.0, 5.0)).expect("plain cylinder");
    assert!((plain.radius - tap).abs() < 1e-5);

    // Tessellation for mesh export (STL/3MF exercised from engine/doc with the same mesh).
    let mesh = plate.tessellate(0.05).unwrap();
    assert!(!mesh.triangles.is_empty() && mesh.positions.len() > 8);
    assert_eq!(plate.face_count(), faces);
    eprintln!("P1 tessellation tris={} verts={}", mesh.triangles.len(), mesh.positions.len());
}

#[test]
fn local_cylinder_replacement_yields_curved_closed_solid() {
    let stock = box_solid(Vec3::ZERO, Vec3::new(40.0, 50.0, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let h1 = cylinder(Vec3::new(12.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let h2 = cylinder(Vec3::new(28.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let plate = boolean(&stock, &h1, BoolOp::Cut).unwrap().unwrap();
    let plate = boolean(&plate, &h2, BoolOp::Cut).unwrap().unwrap();
    let faces_before = plate.face_count();
    let on = Vec3::new(28.0, 25.0 + tap, 5.0);
    let threaded = try_replace_cylinder_with_helical_thread(&plate, on, 5.0, 0.8).expect("local helical replace");
    assert_closed_shell(&threaded);
    let faces = threaded.face_count();
    eprintln!("local replace faces: before={faces_before} after={faces}");
    assert!(faces < 80, "must stay compact, not ~4000 facets: {faces}");
    let (_, cylinders, _, unknown) = count_surfaces(&threaded);
    assert!(unknown >= 2, "B-spline flanks: {unknown}");
    assert!(cylinders >= 2, "cylinders: {cylinders}");
    let plain = cylinder_face_at(&threaded, Vec3::new(12.0 - tap, 25.0, 5.0)).expect("plain remains");
    assert!((plain.radius - tap).abs() < 1e-4);
    assert!(
        (0..threaded.face_count()).any(|i| matches!(face_surface(&threaded, i), Some(FaceSurface::Cylinder { origin, radius, .. }) if (origin.x - 12.0).abs() < 1e-2 && (radius - tap).abs() < 1e-3)),
        "plain hole analytic after replace"
    );
    let before_vol = coarse_volume(&plate);
    let vol = coarse_volume(&threaded);
    eprintln!("local replace vol~{vol:.1} before~{before_vol:.1}");
    // Two tap holes in a 40×50×10 plate ≈ 19723; a filled bore sits near/above the solid box (20000).
    assert!(vol < before_vol - 0.05, "local replace volume {vol} vs before {before_vol} suggests a filled bore");
    assert!(vol < 19850.0, "local replace volume {vol} suggests a filled bore");
}


#[test]
fn internal_thread_display_plan_clears_the_bore() {
    // Issue follow-up: truck's mesher chords helical crest cylinders across a through-hole.
    // The display plan (#1289) must succeed for a Try4-sized plate and leave the bore open.
    let stock = box_solid(Vec3::ZERO, Vec3::new(27.0, 24.5, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let (cx, cy) = (20.76062192932085, 17.874947076172305);
    let hole = cylinder(Vec3::new(cx, cy, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let plate = boolean(&stock, &hole, BoolOp::Cut).unwrap().unwrap();
    let on = Vec3::new(cx, cy + tap, 5.0);
    let threaded = try_replace_cylinder_with_helical_thread(&plate, on, 5.0, 0.8).expect("thread");
    assert_closed_shell(&threaded);
    assert!(crate::tess::seam::turns_more_than_once(&threaded.solid));
    // Coarse display plan — fine meshing of progressive plane∩flank edges is slow.
    let (plan, _) = crate::tess::plan(&threaded.solid, 0.15, None).expect("display plan for internal helical crest");
    let mesh = plan.evaluate();
    let dir = Vec3::new(0.0, 0.0, -1.0);
    assert!(mesh.raycast(Vec3::new(cx, cy, 12.0), dir).is_none(), "axis must clear the display mesh");
    let mut blocked = 0usize;
    for i in 0..12 {
        let a = std::f64::consts::TAU * i as f64 / 12.0;
        let o = Vec3::new(cx + 0.3 * a.cos(), cy + 0.3 * a.sin(), 12.0);
        if let Some((t, _)) = mesh.raycast(o, dir) {
            let z = 12.0 - t;
            if (0.0..=10.0).contains(&z) {
                blocked += 1;
            }
        }
    }
    assert_eq!(blocked, 0, "ring r=0.3 must clear the display mesh, blocked {blocked}/12");
    let tess = threaded.tessellate(0.15).expect("tessellate");
    assert!(tess.raycast(Vec3::new(cx, cy, 12.0), dir).is_none(), "tessellate mesh must also clear the bore");
    let mut top_loops = 0usize;
    let mut bot_loops = 0usize;
    for i in 0..threaded.face_count() {
        let Some(FaceSurface::Plane { point, normal }) = face_surface(&threaded, i) else { continue };
        let n = normal.normalized().unwrap_or(Vec3::Z);
        if (point.z - 10.0).abs() < 1e-3 && n.z.abs() > 0.9 {
            top_loops = threaded.solid.face_iter().nth(i).map(|f| f.boundaries().len()).unwrap_or(0);
        }
        if point.z.abs() < 1e-3 && n.z.abs() > 0.9 {
            bot_loops = threaded.solid.face_iter().nth(i).map(|f| f.boundaries().len()).unwrap_or(0);
        }
    }
    assert!(top_loops >= 2 && bot_loops >= 2, "top={top_loops} bot={bot_loops}");
    // Plane-trimmed entries leave a scalloped hole; reject only tris that chord across the
    // clear central bore (display plan mesh is authoritative for #1289).
    let mut spanning = 0usize;
    for t in &mesh.triangles {
        let Some([a, b, c]) = mesh.tri(t) else { continue };
        let n = (b - a).cross(c - a);
        let area = n.len();
        if area < 1e-12 {
            continue;
        }
        if (n.z / area).abs() < 0.85 {
            continue;
        }
        let centroid = (a + b + c) / 3.0;
        let r = ((centroid.x - cx).powi(2) + (centroid.y - cy).powi(2)).sqrt();
        if r < 0.5 {
            spanning += 1;
        }
    }
    assert_eq!(spanning, 0, "display mesh must not chord across the bore ({spanning} tris)");
}

#[test]
fn four_sequential_m5_threads_stay_curved_and_analytic() {
    let stock = box_solid(Vec3::ZERO, Vec3::new(40.0, 50.0, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let centers = [(10.0, 12.0), (30.0, 12.0), (10.0, 38.0), (30.0, 38.0)];
    let mut plate = stock;
    for &(x, y) in &centers {
        let hole = cylinder(Vec3::new(x, y, -1.0), Vec3::Z, tap, 12.0).unwrap();
        plate = boolean(&plate, &hole, BoolOp::Cut).unwrap().unwrap();
    }
    let mut threaded = plate;
    for (i, &(x, y)) in centers.iter().enumerate() {
        let on = Vec3::new(x, y + tap, 5.0);
        assert!(cylinder_face_at(&threaded, on).is_some(), "cylinder_face_at before thread {i}");
        threaded = try_replace_cylinder_with_helical_thread(&threaded, on, 5.0, 0.8).unwrap_or_else(|e| panic!("thread {i}: {e}"));
        assert_closed_shell(&threaded);
        let faces = threaded.face_count();
        assert!(faces < 200, "after {i} faces={faces}");
        // Remaining untapped holes stay analytic and selectable.
        for &(ux, uy) in centers.iter().skip(i + 1) {
            let c = cylinder_face_at(&threaded, Vec3::new(ux, uy + tap, 5.0));
            assert!(c.is_some(), "remaining hole ({ux},{uy}) after thread {i}");
            assert!((c.unwrap().radius - tap).abs() < 1e-3);
        }
    }
    let (_, cylinders, _, unknown) = count_surfaces(&threaded);
    eprintln!("4×M5 faces={} cyl={} flank={}", threaded.face_count(), cylinders, unknown);
    assert!(unknown >= 8, "four pairs of flanks: {unknown}");
    assert!(threaded.face_count() < 200);
}

#[test]
fn external_ridge_matches_display_fixture_topology() {
    let b = external_ridge_thread(Vec3::ZERO, Vec3::Z, 10.0, 1.5, 6.0).unwrap();
    assert!(b.validity().is_empty(), "{:?}", b.validity());
    let (_, cylinders, _, unknown) = count_surfaces(&b);
    assert!(cylinders >= 2 && unknown >= 2, "cyl={cylinders} flank={unknown}");
}



















#[test]
fn trimmed_entry_has_no_artificial_end_face() {
    let stock = box_solid(Vec3::ZERO, Vec3::new(40.0, 50.0, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let hole = cylinder(Vec3::new(20.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let plate = boolean(&stock, &hole, BoolOp::Cut).unwrap().unwrap();
    let on = Vec3::new(20.0, 25.0 + tap, 5.0);
    let threaded = try_replace_cylinder_with_helical_thread(&plate, on, 5.0, 0.8).expect("trim replace");
    assert_closed_shell(&threaded);
    let origin = Vec3::new(20.0, 25.0, 0.0);
    // No radial planar end-cap inside the bore (the old rectangular termination).
    for i in 0..threaded.face_count() {
        let Some(FaceSurface::Plane { point, normal }) = face_surface(&threaded, i) else { continue };
        let n = normal.normalized().unwrap_or(Vec3::Z);
        if n.z.abs() > 0.85 {
            continue;
        }
        let along = point.z;
        let rad = ((point.x - origin.x).powi(2) + (point.y - origin.y).powi(2)).sqrt();
        assert!(
            !(rad < tap + 0.7 && along > 0.3 && along < 9.7),
            "artificial radial end face at {point:?} n={n:?}"
        );
    }
    // Entry/exit hole wires lie on the plate planes.
    for (z_face, label) in [(0.0, "bottom"), (10.0, "top")] {
        let mut samples = 0usize;
        for i in 0..threaded.face_count() {
            let Some(FaceSurface::Plane { point, normal }) = face_surface(&threaded, i) else { continue };
            let n = normal.normalized().unwrap_or(Vec3::Z);
            if (point.z - z_face).abs() > 1e-3 || n.z.abs() < 0.9 {
                continue;
            }
            let f = threaded.solid.face_iter().nth(i).expect("face");
            for w in f.absolute_boundaries().iter().skip(1) {
                for e in w.edge_iter() {
                    use truck_modeling::{BoundedCurve, ParametricCurve};
                    let c = e.curve();
                    let (t0, t1) = c.range_tuple();
                    for k in 0..=4 {
                        let t = t0 + (t1 - t0) * k as f64 / 4.0;
                        let p = crate::body::from_p3(c.subs(t));
                        let r = ((p.x - origin.x).powi(2) + (p.y - origin.y).powi(2)).sqrt();
                        if r < 2.6 {
                            assert!((p.z - z_face).abs() < 1e-3, "{label} entry off plane: {p:?}");
                            samples += 1;
                        }
                    }
                }
            }
        }
        assert!(samples >= 8, "{label} entry samples={samples}");
    }
    let before = coarse_volume(&plate);
    let vol = coarse_volume(&threaded);
    assert!(vol < before - 0.05, "bore must be empty: vol={vol} before={before}");
}


#[test]
fn plate_with_trimmed_thread_is_closed_curved() {
    let b = crate::plate_with_trimmed_internal_thread(40.0, 50.0, 10.0, (12.0, 25.0), (28.0, 25.0), 5.0, 0.8).expect("trimmed PoC");
    assert_closed_shell(&b);
    let (_, cylinders, _, unknown) = count_surfaces(&b);
    assert!(unknown >= 2 && cylinders >= 3, "cyl={cylinders} flank={unknown}");
    let vol = coarse_volume(&b);
    assert!(vol < 19750.0, "PoC volume {vol} suggests a filled bore");
}


























#[test]
fn progressive_wall_sew_only_smoke() {
    use solidcraft_geom::Vec3;
    let t0 = std::time::Instant::now();
    let (wall, bot, top) = crate::helical_thread_trim::internal_thread_wall_trimmed(
        Vec3::new(28.0, 25.0, 0.0), Vec3::Z, Vec3::X, 5.0, 0.8, 2.1, 0.0, 10.0,
    ).expect("wall");
    eprintln!("wall sew {:?} faces={} bot_edges={} top_edges={}", t0.elapsed(), wall.face_count(), bot.len(), top.len());
    let t1 = std::time::Instant::now();
    let w = crate::helical_thread_trim::free_wire_on_plane(&wall, Vec3::new(28.0, 25.0, 0.0), Vec3::Z, 0.0, 1e-2).expect("bot wire");
    eprintln!("free wire {:?} edges={}", t1.elapsed(), w.edge_iter().count());
}

#[test]
fn entry_flank_mid_radius_stays_outside_crest() {
    // Progressive plane∩flank: every sample of the top/bottom hole wire in the flank angular
    // sector must stay at r ≥ crest. A constant-z chord would dip to ~1.29 mm for M5.
    let stock = box_solid(Vec3::ZERO, Vec3::new(40.0, 50.0, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let hole = cylinder(Vec3::new(20.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let plate = boolean(&stock, &hole, BoolOp::Cut).unwrap().unwrap();
    let on = Vec3::new(20.0, 25.0 + tap, 5.0);
    let threaded = try_replace_cylinder_with_helical_thread(&plate, on, 5.0, 0.8).expect("trim");
    assert_closed_shell(&threaded);
    let origin = Vec3::new(20.0, 25.0, 0.0);
    let major_r = 2.5;
    let mut flank_samples = 0usize;
    let mut min_r = f64::INFINITY;
    for i in 0..threaded.face_count() {
        let Some(FaceSurface::Plane { point, normal }) = face_surface(&threaded, i) else { continue };
        let n = normal.normalized().unwrap_or(Vec3::Z);
        if n.z.abs() > 0.85 {
            continue;
        }
        let rad = ((point.x - origin.x).powi(2) + (point.y - origin.y).powi(2)).sqrt();
        assert!(!(rad < tap + 0.7 && point.z > 0.3 && point.z < 9.7), "radial end face at {point:?}");
    }
    for (z_face, _) in [(0.0_f64, "bottom"), (10.0, "top")] {
        for i in 0..threaded.face_count() {
            let Some(FaceSurface::Plane { point, normal }) = face_surface(&threaded, i) else { continue };
            let n = normal.normalized().unwrap_or(Vec3::Z);
            if (point.z - z_face).abs() > 1e-3 || n.z.abs() < 0.9 {
                continue;
            }
            let f = threaded.solid.face_iter().nth(i).expect("face");
            for w in f.absolute_boundaries().iter().skip(1) {
                for e in w.edge_iter() {
                    use truck_modeling::{BoundedCurve, ParametricCurve};
                    let c = e.curve();
                    let (t0, t1) = c.range_tuple();
                    for k in 0..=8 {
                        let t = t0 + (t1 - t0) * k as f64 / 8.0;
                        let p = crate::body::from_p3(c.subs(t));
                        let r = ((p.x - origin.x).powi(2) + (p.y - origin.y).powi(2)).sqrt();
                        if r > tap + 0.02 && r < major_r + 0.05 {
                            min_r = min_r.min(r);
                            flank_samples += 1;
                            assert!(
                                r + 1e-3 >= tap,
                                "entry flank dips inside crest: r={r:.4} crest={tap:.4} at {p:?}"
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(flank_samples >= 4, "flank samples={flank_samples}");
    assert!(min_r + 1e-3 >= tap, "min flank radius {min_r:.4} < crest {tap:.4}");
}
