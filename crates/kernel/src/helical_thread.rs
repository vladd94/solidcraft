//! Helical ISO metric thread B-rep: ruled B-spline flanks between cubic helix chains, with
//! analytic crest and root cylinders. Faces are sewn; faceted booleans are not used.
//!
//! A circular helix is not a finite-degree NURBS. Helix edges are piecewise cubics; see
//! [`helix_approximation_errors`] for measured radial and axial error at the chosen piece size.

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use solidcraft_geom::Vec3;

use crate::sew::{EdgeSpec, FaceSpec, SurfSpec};
use crate::{Body, KernelError, Result, sew};

/// Maximum angular span (radians) of one cubic Bézier piece of a helix edge.
pub const HELIX_PIECE: f64 = TAU / 16.0;

fn basic_h(pitch: f64) -> f64 {
    pitch * 3.0_f64.sqrt() / 2.0
}

/// Conventional tap-drill radius for common coarse pairs (kernel-local chart for the PoC).
pub fn tap_radius(major: f64, pitch: f64) -> f64 {
    [
        (3.0, 0.5, 2.5),
        (4.0, 0.7, 3.3),
        (5.0, 0.8, 4.2),
        (6.0, 1.0, 5.0),
        (8.0, 1.25, 6.8),
        (10.0, 1.5, 8.5),
    ]
    .into_iter()
    .find(|(n, p, _)| (*n - major).abs() < 1e-9 && (*p - pitch).abs() < 1e-9)
    .map(|(_, _, d)| d * 0.5)
    .unwrap_or((major - pitch) * 0.5)
}

fn helix_edge(origin: Vec3, axis: Vec3, u: Vec3, r: f64, z0: f64, pitch: f64, a0: f64, a1: f64) -> Result<EdgeSpec> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("helix axis".into()))?;
    let u = u.normalized().ok_or_else(|| KernelError::Invalid("helix radial".into()))?;
    let v = k.cross(u);
    if !(r.is_finite() && r > 0.0 && pitch.is_finite() && pitch > 0.0 && a0.is_finite() && a1.is_finite() && (a1 - a0).abs() > 1e-12) {
        return Err(KernelError::Invalid("helix parameters".into()));
    }
    let n = (((a1 - a0).abs() / HELIX_PIECE).ceil() as usize).max(1);
    let h = (a1 - a0) / n as f64;
    let kappa = 4.0 / 3.0 * (h / 4.0).tan();
    let at = |a: f64| origin + k * (z0 + pitch * a / TAU) + (u * a.cos() + v * a.sin()) * r;
    let rise = k * (pitch * h / TAU / 3.0);
    let turn = |a: f64| (u * (-a.sin()) + v * a.cos()) * (r * kappa);
    let mut ctrl = vec![at(a0)];
    for i in 0..n {
        let (s, e) = (a0 + h * i as f64, a0 + h * (i + 1) as f64);
        ctrl.extend([at(s) + turn(s) + rise, at(e) - turn(e) - rise, at(e)]);
    }
    Ok(EdgeSpec::Chain { degree: 3, ctrl: ctrl.into() })
}

fn rev(e: &EdgeSpec) -> EdgeSpec {
    match e {
        EdgeSpec::Chain { degree, ctrl } => EdgeSpec::Chain { degree: *degree, ctrl: ctrl.iter().rev().copied().collect::<Arc<[Vec3]>>() },
        EdgeSpec::Line { a, b } => EdgeSpec::Line { a: *b, b: *a },
        EdgeSpec::Arc { a, b, mid } => EdgeSpec::Arc { a: *b, b: *a, mid: *mid },
    }
}

fn line(a: Vec3, b: Vec3) -> EdgeSpec {
    EdgeSpec::Line { a, b }
}

fn ends(e: &EdgeSpec) -> Result<(Vec3, Vec3)> {
    match e {
        EdgeSpec::Chain { ctrl, .. } => Ok((
            *ctrl.first().ok_or_else(|| KernelError::Invalid("empty helix".into()))?,
            *ctrl.last().ok_or_else(|| KernelError::Invalid("empty helix".into()))?,
        )),
        EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => Ok((*a, *b)),
    }
}

fn circle(origin: Vec3, axis: Vec3, u: Vec3, r: f64, z: f64) -> Result<Vec<EdgeSpec>> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("circle axis".into()))?;
    let u = u.normalized().ok_or_else(|| KernelError::Invalid("circle radial".into()))?;
    let v = k.cross(u);
    let p = |a: f64| origin + k * z + (u * a.cos() + v * a.sin()) * r;
    Ok(vec![
        EdgeSpec::Arc { a: p(0.3), b: p(0.3 + PI), mid: p(0.3 + PI / 2.0) },
        EdgeSpec::Arc { a: p(0.3 + PI), b: p(0.3), mid: p(0.3 + 1.5 * PI) },
    ])
}

fn rect(z: f64, length: f64, width: f64) -> Vec<EdgeSpec> {
    let (a, b, c, d) = (Vec3::new(0.0, 0.0, z), Vec3::new(length, 0.0, z), Vec3::new(length, width, z), Vec3::new(0.0, width, z));
    vec![line(a, b), line(b, c), line(c, d), line(d, a)]
}

/// Max radial deviation from the cylinder of radius `r`, and max axial deviation from the
/// linear pitch law `z = pitch·θ/(2π)`, for one turn of piecewise-cubic helices of span `piece`.
pub fn helix_approximation_errors(r: f64, pitch: f64, piece: f64) -> Result<(f64, f64)> {
    if !(r > 0.0 && pitch > 0.0 && piece > 0.0 && piece <= TAU) {
        return Err(KernelError::Invalid("helix error sample".into()));
    }
    let n = ((TAU / piece).ceil() as usize).max(1);
    let h = TAU / n as f64;
    let kappa = 4.0 / 3.0 * (h / 4.0).tan();
    let at = |a: f64| Vec3::new(r * a.cos(), r * a.sin(), pitch * a / TAU);
    let rise = Vec3::new(0.0, 0.0, pitch * h / TAU / 3.0);
    let turn = |a: f64| Vec3::new(-a.sin(), a.cos(), 0.0) * (r * kappa);
    let mut ctrl = vec![at(0.0)];
    for i in 0..n {
        let (s, e) = (h * i as f64, h * (i + 1) as f64);
        ctrl.extend([at(s) + turn(s) + rise, at(e) - turn(e) - rise, at(e)]);
    }
    let mut max_radial = 0.0_f64;
    let mut max_axial = 0.0_f64;
    let pieces = (ctrl.len() - 1) / 3;
    for i in 0..pieces {
        let (c0, c1, c2, c3) = (ctrl[3 * i], ctrl[3 * i + 1], ctrl[3 * i + 2], ctrl[3 * i + 3]);
        for s in 0..=64 {
            let t = s as f64 / 64.0;
            let mt = 1.0 - t;
            let p = c0 * (mt * mt * mt) + c1 * (3.0 * mt * mt * t) + c2 * (3.0 * mt * t * t) + c3 * (t * t * t);
            let radial = (p.x * p.x + p.y * p.y).sqrt();
            max_radial = max_radial.max((radial - r).abs());
            let a_param = h * (i as f64 + t);
            max_axial = max_axial.max((p.z - pitch * a_param / TAU).abs());
        }
    }
    Ok((max_radial, max_axial))
}

/// External ISO metric ridge (bolt), generalized from the display fixture: 60° flanks, P/8 crest,
/// core at the minor diameter. Right-handed about `axis` through `origin`.
pub fn external_ridge_thread(origin: Vec3, axis: Vec3, major: f64, pitch: f64, turns: f64) -> Result<Body> {
    if !(major > 0.0 && pitch > 0.0 && turns > 0.0 && turns <= 190.0 && origin.is_finite()) {
        return Err(KernelError::Invalid("thread size".into()));
    }
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("thread axis".into()))?;
    let u = k.any_perp();
    let h = basic_h(pitch);
    let (rc, rm) = (major / 2.0 - 5.0 * h / 8.0, major / 2.0);
    let (foot, crest) = ((pitch - pitch / 4.0) / 2.0, pitch / 16.0);
    let z0 = pitch;
    let len = z0 + pitch * turns + pitch;
    let end = TAU * turns;
    let fl = helix_edge(origin, k, u, rc, z0 - foot, pitch, 0.0, end)?;
    let fu = helix_edge(origin, k, u, rc, z0 + foot, pitch, 0.0, end)?;
    let cl = helix_edge(origin, k, u, rm, z0 - crest, pitch, 0.0, end)?;
    let cu = helix_edge(origin, k, u, rm, z0 + crest, pitch, 0.0, end)?;
    let (fl0, fl1) = ends(&fl)?;
    let (fu0, fu1) = ends(&fu)?;
    let (cl0, cl1) = ends(&cl)?;
    let (cu0, cu1) = ends(&cu)?;
    let v = k.cross(u);
    let at = |r: f64, a: f64, z: f64| origin + k * z + (u * a.cos() + v * a.sin()) * r;
    let out = |a: f64| u * a.cos() + v * a.sin();
    let mid = z0 + pitch / 2.0;
    let flank = 0.5_f64;
    sew(&[
        FaceSpec { loops: vec![circle(origin, k, u, rc, 0.0)?.into_iter().rev().map(|e| rev(&e)).collect()], surface: SurfSpec::Plane, probe: None },
        FaceSpec { loops: vec![circle(origin, k, u, rc, len)?], surface: SurfSpec::Plane, probe: None },
        FaceSpec {
            loops: vec![
                circle(origin, k, u, rc, 0.0)?,
                circle(origin, k, u, rc, len)?.into_iter().rev().map(|e| rev(&e)).collect(),
                vec![line(fl0, fu0), fu.clone(), line(fu1, fl1), rev(&fl)],
            ],
            surface: SurfSpec::Revolved {
                a: origin + u * rc - k,
                b: origin + u * rc + k * (len + 1.0),
                axis_origin: origin,
                axis: k,
                angle: TAU,
            },
            probe: Some((at(rc, PI, z0 * 0.5), out(PI))),
        },
        FaceSpec {
            loops: vec![vec![fl.clone(), line(fl1, cl1), rev(&cl), line(cl0, fl0)]],
            surface: SurfSpec::Ruled { a: fl.clone(), b: cl.clone() },
            probe: Some((at((rc + rm) / 2.0, PI, mid - (foot + crest) / 2.0), out(PI) * flank - k * 0.866)),
        },
        FaceSpec {
            loops: vec![vec![cu.clone(), line(cu1, fu1), rev(&fu), line(fu0, cu0)]],
            surface: SurfSpec::Ruled { a: fu.clone(), b: cu.clone() },
            probe: Some((at((rc + rm) / 2.0, PI, mid + (foot + crest) / 2.0), out(PI) * flank + k * 0.866)),
        },
        FaceSpec {
            loops: vec![vec![cl.clone(), line(cl1, cu1), rev(&cu), line(cu0, cl0)]],
            surface: SurfSpec::Revolved {
                a: origin + u * rm - k,
                b: origin + u * rm + k * (len + 1.0),
                axis_origin: origin,
                axis: k,
                angle: TAU,
            },
            probe: Some((at(rm, PI, mid), out(PI))),
        },
        FaceSpec { loops: vec![vec![line(fl0, cl0), line(cl0, cu0), line(cu0, fu0), line(fu0, fl0)]], surface: SurfSpec::Plane, probe: None },
        FaceSpec { loops: vec![vec![line(fl1, fu1), line(fu1, cu1), line(cu1, cl1), line(cl1, fl1)]], surface: SurfSpec::Plane, probe: None },
    ])
}

/// Internal ISO metric M-series thread in a square plate (side `2·outer`, thickness for `turns`
/// plus lead-in). Threaded hole at the plate centre; a second plain hole sits in the corner so
/// multi-hole analytic preservation can be checked on the same solid. `origin` is the plate
/// corner (min XYZ); `axis` must be +Z for this PoC.
pub fn internal_thread_tube(origin: Vec3, axis: Vec3, major: f64, pitch: f64, turns: f64, outer: f64) -> Result<Body> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("thread axis".into()))?;
    if (k - Vec3::Z).len() > 1e-9 {
        return Err(KernelError::Invalid("internal_thread_tube PoC expects a +Z axis".into()));
    }
    if !(outer > major / 2.0 + pitch && turns >= 1.0) {
        return Err(KernelError::Invalid("internal thread size".into()));
    }
    let side = outer * 2.0;
    let height = pitch * (turns + 2.0);
    let plate = plate_with_plain_and_internal_thread(side, side, height, (outer * 0.4, outer * 0.4), (outer, outer), major, pitch)?;
    crate::transform(&plate, origin, Vec3::ZERO, Vec3::Z, 0.0)
}

/// Rectangular plate (0…L × 0…W × 0…H) with a plain cylindrical through-hole and a second
/// through-hole whose wall is an internal helical ISO thread — all faces sewn in one body.
/// The plain hole remains an analytic cylinder selectable by [`crate::cylinder_face_at`].
pub fn plate_with_plain_and_internal_thread(
    length: f64,
    width: f64,
    height: f64,
    plain_xy: (f64, f64),
    thread_xy: (f64, f64),
    major: f64,
    pitch: f64,
) -> Result<Body> {
    let rc = tap_radius(major, pitch);
    let rm = major / 2.0;
    if !(length > major * 3.0 && width > major * 3.0 && height > pitch * 3.0 && rc > 0.5 && rm > rc) {
        return Err(KernelError::Invalid("plate / thread size".into()));
    }
    let turns = (height / pitch).floor().max(2.0) - 1.0;
    if turns < 1.0 {
        return Err(KernelError::Invalid("plate too thin for a turn".into()));
    }
    let k = Vec3::Z;
    let u = Vec3::X;
    let origin_t = Vec3::new(thread_xy.0, thread_xy.1, 0.0);
    let origin_p = Vec3::new(plain_xy.0, plain_xy.1, 0.0);
    let h = basic_h(pitch);
    let basic = 5.0 * h / 8.0;
    if rm - rc < basic / 4.0 {
        return Err(KernelError::Invalid("thread profile too shallow".into()));
    }
    let (foot, crest) = ((pitch - pitch / 4.0) / 2.0, pitch / 16.0);
    let z0 = pitch * 0.5;
    // Start at π/2 so the radial end-cap planes are x ≈ thread_xy.0 (not y = thread_xy.1),
    // leaving the plain hole clear for cylinder_face_at.
    let a0 = PI / 2.0;
    let end = a0 + TAU * turns;
    let fl = helix_edge(origin_t, k, u, rc, z0 - foot, pitch, a0, end)?;
    let fu = helix_edge(origin_t, k, u, rc, z0 + foot, pitch, a0, end)?;
    let cl = helix_edge(origin_t, k, u, rm, z0 - crest, pitch, a0, end)?;
    let cu = helix_edge(origin_t, k, u, rm, z0 + crest, pitch, a0, end)?;
    let (fl0, fl1) = ends(&fl)?;
    let (fu0, fu1) = ends(&fu)?;
    let (cl0, cl1) = ends(&cl)?;
    let (cu0, cu1) = ends(&cu)?;
    let v = k.cross(u);
    let at_t = |r: f64, a: f64, z: f64| origin_t + k * z + (u * a.cos() + v * a.sin()) * r;
    let out_t = |a: f64| u * a.cos() + v * a.sin();
    let mid = z0 + pitch / 2.0;
    let flank = 0.5_f64;
    let (l, w, hz) = (length, width, height);

    // Outer box + two holes on top/bottom. Plain hole is a revolved cylinder. Threaded hole uses
    // crest/root cylinders and ruled flanks (same topology as the tube, embedded in the plate).
    let mut faces = vec![
        // Bottom: outer rect, plain hole, threaded crest hole
        FaceSpec {
            loops: vec![
                rect(0.0, l, w).into_iter().rev().map(|e| rev(&e)).collect(),
                circle(origin_p, k, u, rc, 0.0)?,
                circle(origin_t, k, u, rc, 0.0)?,
            ],
            surface: SurfSpec::Plane,
            probe: None,
        },
        // Top
        FaceSpec {
            loops: vec![
                rect(hz, l, w),
                circle(origin_p, k, u, rc, hz)?.into_iter().rev().map(|e| rev(&e)).collect(),
                circle(origin_t, k, u, rc, hz)?.into_iter().rev().map(|e| rev(&e)).collect(),
            ],
            surface: SurfSpec::Plane,
            probe: None,
        },
        // Four sides
        FaceSpec {
            loops: vec![vec![
                line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(l, 0.0, 0.0)),
                line(Vec3::new(l, 0.0, 0.0), Vec3::new(l, 0.0, hz)),
                line(Vec3::new(l, 0.0, hz), Vec3::new(0.0, 0.0, hz)),
                line(Vec3::new(0.0, 0.0, hz), Vec3::new(0.0, 0.0, 0.0)),
            ]],
            surface: SurfSpec::Plane,
            probe: None,
        },
        FaceSpec {
            loops: vec![vec![
                line(Vec3::new(l, 0.0, 0.0), Vec3::new(l, w, 0.0)),
                line(Vec3::new(l, w, 0.0), Vec3::new(l, w, hz)),
                line(Vec3::new(l, w, hz), Vec3::new(l, 0.0, hz)),
                line(Vec3::new(l, 0.0, hz), Vec3::new(l, 0.0, 0.0)),
            ]],
            surface: SurfSpec::Plane,
            probe: None,
        },
        FaceSpec {
            loops: vec![vec![
                line(Vec3::new(l, w, 0.0), Vec3::new(0.0, w, 0.0)),
                line(Vec3::new(0.0, w, 0.0), Vec3::new(0.0, w, hz)),
                line(Vec3::new(0.0, w, hz), Vec3::new(l, w, hz)),
                line(Vec3::new(l, w, hz), Vec3::new(l, w, 0.0)),
            ]],
            surface: SurfSpec::Plane,
            probe: None,
        },
        FaceSpec {
            loops: vec![vec![
                line(Vec3::new(0.0, w, 0.0), Vec3::new(0.0, 0.0, 0.0)),
                line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, hz)),
                line(Vec3::new(0.0, 0.0, hz), Vec3::new(0.0, w, hz)),
                line(Vec3::new(0.0, w, hz), Vec3::new(0.0, w, 0.0)),
            ]],
            surface: SurfSpec::Plane,
            probe: None,
        },
        // Plain hole wall (analytic cylinder).
        FaceSpec {
            loops: vec![
                circle(origin_p, k, u, rc, 0.0)?.into_iter().rev().map(|e| rev(&e)).collect(),
                circle(origin_p, k, u, rc, hz)?,
            ],
            surface: SurfSpec::Revolved {
                a: origin_p + u * rc - k,
                b: origin_p + u * rc + k * (hz + 1.0),
                axis_origin: origin_p,
                axis: k,
                angle: TAU,
            },
            probe: Some((origin_p + Vec3::new(-rc, 0.0, hz * 0.5), Vec3::new(-1.0, 0.0, 0.0))),
        },
        // Threaded crest cylinder with helical window
        FaceSpec {
            loops: vec![
                circle(origin_t, k, u, rc, 0.0)?.into_iter().rev().map(|e| rev(&e)).collect(),
                circle(origin_t, k, u, rc, hz)?,
                vec![line(fl0, fu0), fu.clone(), line(fu1, fl1), rev(&fl)],
            ],
            surface: SurfSpec::Revolved {
                a: origin_t + u * rc - k,
                b: origin_t + u * rc + k * (hz + 1.0),
                axis_origin: origin_t,
                axis: k,
                angle: TAU,
            },
            probe: Some((at_t(rc, 0.0, z0 * 0.25), -out_t(0.0))),
        },
        FaceSpec {
            loops: vec![vec![fl.clone(), line(fl1, cl1), rev(&cl), line(cl0, fl0)]],
            surface: SurfSpec::Ruled { a: fl.clone(), b: cl.clone() },
            probe: Some((at_t((rc + rm) / 2.0, PI, mid - (foot + crest) / 2.0), out_t(PI) * flank)),
        },
        FaceSpec {
            loops: vec![vec![cu.clone(), line(cu1, fu1), rev(&fu), line(fu0, cu0)]],
            surface: SurfSpec::Ruled { a: fu.clone(), b: cu.clone() },
            probe: Some((at_t((rc + rm) / 2.0, PI, mid + (foot + crest) / 2.0), out_t(PI) * flank)),
        },
        FaceSpec {
            loops: vec![vec![cl.clone(), line(cl1, cu1), rev(&cu), line(cu0, cl0)]],
            surface: SurfSpec::Revolved {
                a: origin_t + u * rm - k,
                b: origin_t + u * rm + k * (hz + 1.0),
                axis_origin: origin_t,
                axis: k,
                angle: TAU,
            },
            probe: Some((at_t(rm, PI, mid), out_t(PI))),
        },
        FaceSpec { loops: vec![vec![line(fl0, cl0), line(cl0, cu0), line(cu0, fu0), line(fu0, fl0)]], surface: SurfSpec::Plane, probe: None },
        FaceSpec { loops: vec![vec![line(fl1, fu1), line(fu1, cu1), line(cu1, cl1), line(cl1, fl1)]], surface: SurfSpec::Plane, probe: None },
    ];
    // Ensure we did not drop faces silently.
    if faces.len() < 12 {
        return Err(KernelError::Failed("plate face list".into()));
    }
    let _ = &mut faces;
    sew(&faces)
}

/// Try to replace one internal cylinder on an existing body with a helical wall by
/// `remove_faces` + `stitch` of a threaded tube. Never falls back to a faceted boolean.
///
/// **PoC blocker:** stitching an open shell (plate minus cylinder) to a closed threaded tube
/// does not yield a single closed solid — the tube brings its own outer wall. A true local
/// face swap needs an open helical-wall shell that shares the bore's rim circles; that path
/// is Stage 3. This function records the failure explicitly.
pub fn try_replace_cylinder_with_helical_thread(body: &Body, on_face: Vec3, major: f64, pitch: f64) -> Result<Body> {
    let cyl = crate::cylinder_face_at(body, on_face).ok_or_else(|| KernelError::Invalid("no cylindrical face there to thread".into()))?;
    if !cyl.internal {
        return Err(KernelError::Invalid("internal helical thread needs a hole".into()));
    }
    let want = tap_radius(major, pitch);
    if (cyl.radius - want).abs() > 0.05 * major {
        return Err(KernelError::Invalid(format!("hole radius {:.3} mm is not tap radius {:.3} mm", cyl.radius, want)));
    }
    let face = crate::surfaces::nearest_face(body, on_face)?;
    let open = crate::remove_faces(body, &[face])?;
    let height = (cyl.end - cyl.start).abs();
    let turns = (height / pitch).floor().max(2.0) - 1.0;
    let tube = internal_thread_tube(Vec3::ZERO, Vec3::Z, major, pitch, turns, major.max(8.0))?;
    match crate::stitch(&[&open, &tube], pitch * 1e-4) {
        Ok(b) if b.validity().is_empty() && b.shell_count() == 1 => Ok(b),
        Ok(_) => Err(KernelError::Failed(
            "local helical sew into an existing plate is not closed yet (stitch returned a non-solid or multi-shell body); use plate_with_plain_and_internal_thread for the curved B-rep proof".into(),
        )),
        Err(e) => Err(KernelError::Failed(format!(
            "local helical sew into an existing plate is not closed yet ({e}); use plate_with_plain_and_internal_thread for the curved B-rep proof"
        ))),
    }
}
