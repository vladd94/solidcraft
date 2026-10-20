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

pub(crate) fn basic_h(pitch: f64) -> f64 {
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
        // Plain hole wall (analytic cylinder). Circles oppose the plate caps for sew; the inward
        // probe flips face orientation (see sew::curved) so absolute boundaries stay seam-cuttable.
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
        // Threaded crest cylinder with helical window.
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

/// Free edges of a surface body (used once).
#[allow(dead_code)]
fn free_truck_edges(body: &Body) -> Vec<truck_modeling::Edge> {
    use truck_modeling as mt;
    let mut uses: std::collections::HashMap<mt::EdgeID, (usize, mt::Edge)> = std::collections::HashMap::new();
    for f in body.solid.face_iter() {
        for e in f.boundaries().iter().flat_map(|w| w.edge_iter()) {
            uses.entry(e.id()).or_insert((0, e.clone())).0 += 1;
        }
    }
    uses.into_values().filter(|(n, _)| *n == 1).map(|(_, e)| e).collect()
}

/// Rim loop of a cylindrical hole at axial parameter `z` (along `axis` from `origin`), taken from
/// the body's free edges so stitch can rematch the plate after `remove_faces`.
#[allow(dead_code)]
fn free_rim_loop(body: &Body, origin: Vec3, axis: Vec3, r: f64, z: f64, tol: f64) -> Result<Vec<EdgeSpec>> {
    use truck_modeling::{BoundedCurve, ParametricCurve};
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("rim axis".into()))?;
    let on_rim = |p: Vec3| {
        let along = (p - origin).dot(k);
        let radial = ((p - origin) - k * along).len();
        (along - z).abs() <= tol && (radial - r).abs() <= tol
    };
    let mut arcs: Vec<(Vec3, Vec3, Vec3)> = Vec::new();
    for e in free_truck_edges(body) {
        let c = e.curve();
        let (t0, t1) = c.range_tuple();
        if !(t0.is_finite() && t1.is_finite()) {
            continue;
        }
        let (a, b, mid) = (crate::body::from_p3(c.subs(t0)), crate::body::from_p3(c.subs(t1)), crate::body::from_p3(c.subs(0.5 * (t0 + t1))));
        if on_rim(a) && on_rim(b) && on_rim(mid) {
            arcs.push((a, b, mid));
        }
    }
    if arcs.is_empty() {
        return Err(KernelError::Failed("no free rim edges at the cylinder end".into()));
    }
    // Order into a single closed loop.
    let mut ordered: Vec<EdgeSpec> = Vec::with_capacity(arcs.len());
    let (a0, b0, m0) = arcs.swap_remove(0);
    ordered.push(EdgeSpec::Arc { a: a0, b: b0, mid: m0 });
    let mut cur = b0;
    while !arcs.is_empty() {
        let mut found = None;
        for (i, (a, b, mid)) in arcs.iter().enumerate() {
            if a.dist(cur) <= tol {
                found = Some((i, EdgeSpec::Arc { a: *a, b: *b, mid: *mid }, *b));
                break;
            }
            if b.dist(cur) <= tol {
                found = Some((i, EdgeSpec::Arc { a: *b, b: *a, mid: *mid }, *a));
                break;
            }
        }
        let Some((i, edge, next)) = found else {
            return Err(KernelError::Failed("rim edges do not form a closed loop".into()));
        };
        arcs.swap_remove(i);
        ordered.push(edge);
        cur = next;
    }
    if cur.dist(a0) > tol {
        return Err(KernelError::Failed("rim loop does not close".into()));
    }
    Ok(ordered)
}

/// Open helical wall for an internal through-hole: crest/root cylinders + ruled B-spline flanks.
/// Outer free boundaries are the supplied top/bottom rim loops (from the plate after cylinder removal).
/// `crest` is the hole wall radius (tap drill / minor); root is the major radius.
#[allow(dead_code)]
fn internal_thread_wall_open(
    origin: Vec3,
    axis: Vec3,
    u: Vec3,
    major: f64,
    pitch: f64,
    crest: f64,
    z_lo: f64,
    z_hi: f64,
    bot: Vec<EdgeSpec>,
    top: Vec<EdgeSpec>,
) -> Result<Body> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("thread axis".into()))?;
    let u = u.normalized().ok_or_else(|| KernelError::Invalid("thread radial".into()))?;
    let rc = crest;
    let rm = major / 2.0;
    let height = z_hi - z_lo;
    if !(height > pitch * 2.0 && rm > rc && rc > 0.0) {
        return Err(KernelError::Invalid("thread height / diameter".into()));
    }
    let turns = (height / pitch).floor().max(2.0) - 1.0;
    if turns < 1.0 {
        return Err(KernelError::Invalid("too few turns".into()));
    }
    let h = basic_h(pitch);
    let basic = 5.0 * h / 8.0;
    if rm - rc < basic / 4.0 {
        return Err(KernelError::Invalid("thread profile too shallow".into()));
    }
    let (foot, crest) = ((pitch - pitch / 4.0) / 2.0, pitch / 16.0);
    let z0 = z_lo + pitch * 0.5;
    let a0 = PI / 2.0;
    let end = a0 + TAU * turns;
    let fl = helix_edge(origin, k, u, rc, z0 - foot, pitch, a0, end)?;
    let fu = helix_edge(origin, k, u, rc, z0 + foot, pitch, a0, end)?;
    let cl = helix_edge(origin, k, u, rm, z0 - crest, pitch, a0, end)?;
    let cu = helix_edge(origin, k, u, rm, z0 + crest, pitch, a0, end)?;
    let (fl0, fl1) = ends(&fl)?;
    let (fu0, fu1) = ends(&fu)?;
    let (cl0, cl1) = ends(&cl)?;
    let (cu0, cu1) = ends(&cu)?;
    let v = k.cross(u);
    let at = |r: f64, a: f64, z: f64| origin + k * z + (u * a.cos() + v * a.sin()) * r;
    let out = |a: f64| u * a.cos() + v * a.sin();
    let mid = z0 + pitch / 2.0;
    let flank = 0.5_f64;
    // Absolute free-rim chains are typically CW about +k at both ends. Reverse the bottom only
    // (same as full-sew rev(circle)/circle). Stitch then flips the open wall as a whole so its
    // sew_open normals agree with the plate — leaving an empty through-bore.
    let bot_rev: Vec<EdgeSpec> = bot.iter().rev().map(rev).collect();
    crate::sew_open(&[
        FaceSpec {
            loops: vec![bot_rev, top, vec![line(fl0, fu0), fu.clone(), line(fu1, fl1), rev(&fl)]],
            surface: SurfSpec::Revolved {
                a: origin + u * rc + k * z_lo,
                b: origin + u * rc + k * z_hi,
                axis_origin: origin,
                axis: k,
                angle: TAU,
            },
            probe: Some((at(rc, 0.0, z_lo + (z0 - z_lo) * 0.25), -out(0.0))),
        },
        FaceSpec {
            loops: vec![vec![fl.clone(), line(fl1, cl1), rev(&cl), line(cl0, fl0)]],
            surface: SurfSpec::Ruled { a: fl.clone(), b: cl.clone() },
            probe: Some((at((rc + rm) / 2.0, PI, mid - (foot + crest) / 2.0), out(PI) * flank)),
        },
        FaceSpec {
            loops: vec![vec![cu.clone(), line(cu1, fu1), rev(&fu), line(fu0, cu0)]],
            surface: SurfSpec::Ruled { a: fu.clone(), b: cu.clone() },
            probe: Some((at((rc + rm) / 2.0, PI, mid + (foot + crest) / 2.0), out(PI) * flank)),
        },
        FaceSpec {
            loops: vec![vec![cl.clone(), line(cl1, cu1), rev(&cu), line(cu0, cl0)]],
            surface: SurfSpec::Revolved {
                a: origin + u * rm + k * z_lo,
                b: origin + u * rm + k * z_hi,
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

/// Replace one internal through-hole cylinder with a curved helical thread wall by
/// `remove_faces` + `sew_open` + `stitch`. Never falls back to a faceted boolean.
///
/// The helical groove is built as a continuous helix that extends past the plate faces and is
/// trimmed by those planes: free boundaries are planar entry loops (long crest arc + flanks +
/// short root). Radial planar end-caps are not added. The planar top/bottom faces are rebuilt so
/// their hole wires match those entry loops.
///
/// Supported today: internal cylindrical through-holes whose radius matches the conventional
/// tap drill for `(major, pitch)`. Partial-length threads and external shafts return an
/// explicit error so callers do not silently facet.
pub fn try_replace_cylinder_with_helical_thread(body: &Body, on_face: Vec3, major: f64, pitch: f64) -> Result<Body> {
    let cyl = crate::cylinder_face_at(body, on_face).ok_or_else(|| KernelError::Invalid("no cylindrical face there to thread".into()))?;
    if !cyl.internal {
        return Err(KernelError::Invalid(
            "curved helical B-rep threading supports internal through-holes only for now; external shafts are not supported yet".into(),
        ));
    }
    let height = (cyl.end - cyl.start).abs();
    if height / pitch > 190.0 {
        return Err(KernelError::Invalid(format!("a modelled thread is limited to 190 turns ({:.0} mm at this pitch)", 190.0 * pitch)));
    }
    let h = basic_h(pitch);
    let basic = 5.0 * h / 8.0;
    if major / 2.0 - cyl.radius < basic / 4.0 {
        return Err(KernelError::Invalid(format!(
            "hole radius {:.3} mm is too wide for a modelled M{major}×{pitch} thread (need at least {:.3} mm of profile depth)",
            cyl.radius,
            basic / 4.0
        )));
    }
    // A boolean-cut hole is often two half-cylinders; remove every face of this bore.
    // Build the helical wall in an axis frame where `k` points along increasing world Z when
    // possible. `cylinder_face_at` often returns `k = -Z` for an internal bore; sewing the open
    // wall in that frame leaves a consistently oriented but inverted cavity after stitch
    // (mid-bore solid, volume above the plate).
    let mut k = *cyl.axis;
    let origin = cyl.axis_point;
    let (mut z_lo, mut z_hi) = if cyl.start <= cyl.end { (cyl.start, cyl.end) } else { (cyl.end, cyl.start) };
    if k.dot(Vec3::Z) < 0.0 {
        k = -k;
        let (a, b) = (-z_hi, -z_lo);
        z_lo = a;
        z_hi = b;
    }
    let face_tol = (pitch * 1e-2).max(1e-3);
    let mut bore_faces: Vec<usize> = Vec::new();
    for i in 0..body.face_count() {
        let Some(crate::FaceSurface::Cylinder { origin: o, axis, radius }) = crate::face_surface(body, i) else {
            continue;
        };
        let axis = *axis;
        if (radius - cyl.radius).abs() > face_tol {
            continue;
        }
        if axis.cross(k).len() > 1e-6 {
            continue;
        }
        let radial = (o - origin) - k * (o - origin).dot(k);
        if radial.len() > face_tol {
            continue;
        }
        bore_faces.push(i);
    }
    if bore_faces.is_empty() {
        let face = crate::surfaces::nearest_face(body, on_face)?;
        bore_faces.push(face);
    }
    let rim_tol = (pitch * 1e-2).max(1e-3);
    let (bot_face, top_face) = crate::helical_thread_trim::cap_faces_for_bore(body, origin, k, cyl.radius, z_lo, z_hi, rim_tol)?;
    let bot_mt = body.solid.face_iter().nth(bot_face).cloned().ok_or_else(|| KernelError::Failed("bottom face".into()))?;
    let top_mt = body.solid.face_iter().nth(top_face).cloned().ok_or_else(|| KernelError::Failed("top face".into()))?;
    let bot_normal = match crate::face_surface(body, bot_face) {
        Some(crate::FaceSurface::Plane { normal, .. }) => normal.normalized().unwrap_or(-k),
        _ => -k,
    };
    let top_normal = match crate::face_surface(body, top_face) {
        Some(crate::FaceSurface::Plane { normal, .. }) => normal.normalized().unwrap_or(k),
        _ => k,
    };
    let mut drop: Vec<usize> = bore_faces;
    drop.push(bot_face);
    drop.push(top_face);
    drop.sort_unstable();
    drop.dedup();
    let open = crate::remove_faces(body, &drop)?;
    let u = k.any_perp();
    // Mandatory topology audit of the wall FaceSpecs before any sew/stitch.
    let wall_specs =
        crate::helical_thread_trim::internal_thread_wall_face_specs(origin, k, u, major, pitch, cyl.radius, z_lo, z_hi)?;
    let wall = crate::sew_open(&wall_specs.faces)?;
    let bot_entry = wall_specs.bot_entry;
    let top_entry = wall_specs.top_entry;
    // Topology audit of the Boolean remnant vs fixture: free-edge matching is complete
    // (wall↔caps 8/8). Opposite-parity entry reverses (`br != tr`) leave a Regular open
    // shell; Truck `validity()` on those opens is extremely expensive (tens of seconds) and
    // was the perceived "stitch hang". Same-parity reverses close the shell. Prefer those
    // first; only then try the normal-derived pair. Never call `validity()` until the body
    // is a single non-surface solid, and never tessellate for volume here.
    let prefer_bot = bot_normal.dot(k) > 0.0;
    let prefer_top = top_normal.dot(k) > 0.0;
    let mut order = vec![
        (true, true),
        (false, false),
        (prefer_bot, prefer_top),
        (!prefer_bot, prefer_top),
        (prefer_bot, !prefer_top),
        (!prefer_bot, !prefer_top),
    ];
    order.dedup();
    let mut last_err = KernelError::Failed("helical thread stitch produced no closed solid".into());
    let mid = origin + k * (0.5 * (z_lo + z_hi));
    for (br, tr) in order {
        let bottom = match crate::helical_thread_trim::replace_bore_wire_on_face(&bot_mt, origin, k, cyl.radius, bot_entry.clone(), br, rim_tol) {
            Ok(b) => b,
            Err(e) => {
                last_err = e;
                continue;
            }
        };
        let top = match crate::helical_thread_trim::replace_bore_wire_on_face(&top_mt, origin, k, cyl.radius, top_entry.clone(), tr, rim_tol) {
            Ok(b) => b,
            Err(e) => {
                last_err = e;
                continue;
            }
        };
        match crate::stitch(&[&open, &wall, &bottom, &top], pitch * 1e-4) {
            Ok(b) if !b.is_surface() && b.shell_count() == 1 => {
                // Closed solid — safe to validate. Skip tess/volume (expensive on plane∩ flanks).
                if !b.validity().is_empty() {
                    last_err = KernelError::Failed(format!(
                        "helical thread stitch closed but validity failed: {:?}",
                        b.validity()
                    ));
                    continue;
                }
                if !b.contains(mid) {
                    return Ok(b);
                }
                if let Ok(inv) = invert_shell(&b) {
                    if !inv.is_surface() && inv.shell_count() == 1 && inv.validity().is_empty() && !inv.contains(mid) {
                        return Ok(inv);
                    }
                }
                last_err = KernelError::Failed("helical thread stitch closed but filled the bore".into());
            }
            Ok(b) => {
                // Open / multi-shell: do not call validity() — it stalls on Regular opens.
                last_err = KernelError::Failed(format!(
                    "helical thread stitch did not yield a single closed solid (shells={}, surface={})",
                    b.shell_count(),
                    b.is_surface()
                ));
            }
            Err(e) => last_err = KernelError::Failed(format!("helical thread stitch failed: {e}")),
        }
    }
    Err(last_err)
}

fn invert_shell(body: &Body) -> Result<Body> {
    use truck_modeling as mt;
    let faces: Vec<mt::Face> = body.solid.face_iter().map(|f| f.inverse()).collect();
    let shell = mt::Shell::from(faces);
    if shell.shell_condition() != mt::ShellCondition::Closed {
        return Err(KernelError::Failed("inverted shell is not closed".into()));
    }
    Body::new(mt::Solid::new_unchecked(vec![shell]))
}

/// External curved helical thread on a shaft cylinder. Not supported yet — returns an explicit
/// error (legacy faceted path must not be used silently).
pub fn try_replace_external_cylinder_with_helical_thread(body: &Body, on_face: Vec3, major: f64, pitch: f64) -> Result<Body> {
    let _ = (body, on_face, major, pitch);
    Err(KernelError::Invalid(
        "curved helical B-rep threading does not support external shafts yet; use a cosmetic thread or wait for external sew support".into(),
    ))
}
