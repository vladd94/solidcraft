//! Stage-2 proof: curved helical B-rep threads (ruled B-spline flanks), not faceted booleans.

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

#[test]
fn piecewise_cubic_helix_errors_are_documented() {
    for (r, pitch) in [(2.1_f64, 0.8_f64), (2.5, 0.8), (4.0, 1.25)] {
        let (radial, axial) = helix_approximation_errors(r, pitch, HELIX_PIECE).unwrap();
        assert!(radial < 1e-3, "r={r}: radial error {radial} mm");
        assert!(axial < 1e-3, "r={r}: axial pitch error {axial} mm");
        assert!(axial < 0.01 * pitch, "r={r}: axial {axial} vs 1% of pitch");
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
    // Exact surface identity (not a faceted stand-in).
    let plain_exact = (0..plate.face_count()).find_map(|i| match face_surface(&plate, i) {
        Some(FaceSurface::Cylinder { origin, radius, .. }) if (origin.x - 12.0).abs() < 1e-3 && (radius - tap).abs() < 1e-4 => Some(i),
        _ => None,
    });
    assert!(plain_exact.is_some(), "plain hole remains an analytic FaceSurface::Cylinder");
}

#[test]
fn local_cylinder_replacement_reports_blocker_without_faceting() {
    let stock = box_solid(Vec3::ZERO, Vec3::new(40.0, 50.0, 10.0)).unwrap();
    let tap = tap_radius(5.0, 0.8);
    let h1 = cylinder(Vec3::new(12.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let h2 = cylinder(Vec3::new(28.0, 25.0, -1.0), Vec3::Z, tap, 12.0).unwrap();
    let plate = boolean(&stock, &h1, BoolOp::Cut).unwrap().unwrap();
    let plate = boolean(&plate, &h2, BoolOp::Cut).unwrap().unwrap();
    let faces_before = plate.face_count();
    let on = Vec3::new(28.0, 25.0 + tap, 5.0);
    let err = try_replace_cylinder_with_helical_thread(&plate, on, 5.0, 0.8).unwrap_err();
    let msg = err.to_string();
    assert!(msg.contains("not closed yet") || msg.contains("local helical"), "{msg}");
    assert_eq!(plate.face_count(), faces_before);
    assert!(cylinder_face_at(&plate, Vec3::new(12.0, 25.0 + tap, 5.0)).is_some());
}

#[test]
fn external_ridge_matches_display_fixture_topology() {
    let b = external_ridge_thread(Vec3::ZERO, Vec3::Z, 10.0, 1.5, 6.0).unwrap();
    assert!(b.validity().is_empty(), "{:?}", b.validity());
    let (_, cylinders, _, unknown) = count_surfaces(&b);
    assert!(cylinders >= 2 && unknown >= 2, "cyl={cylinders} flank={unknown}");
}
