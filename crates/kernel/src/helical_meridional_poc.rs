//! Experimental minimal PoC: one meridional (same-angle) helical flank with a plane∩flank
//! entry curve shared by the flank and a planar face, sewn into a tiny closed shell.
//!
//! Not wired into production `try_replace` / Hole tooling. See agent Option-1 approval.

use std::f64::consts::TAU;
use std::sync::Arc;

use solidcraft_geom::Vec3;

use crate::sew::{EdgeSpec, FaceSpec, SurfSpec};
use crate::{Body, KernelError, Result};
use crate::helical_thread::HELIX_PIECE;

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

fn helix_point(origin: Vec3, k: Vec3, u: Vec3, r: f64, z0: f64, pitch: f64, a: f64) -> Vec3 {
    let v = k.cross(u);
    origin + k * (z0 + pitch * a / TAU) + (u * a.cos() + v * a.sin()) * r
}

fn angle_at_z(z0: f64, pitch: f64, z: f64) -> f64 {
    TAU * (z - z0) / pitch
}

fn helix_edge(origin: Vec3, k: Vec3, u: Vec3, r: f64, z0: f64, pitch: f64, a0: f64, a1: f64) -> Result<EdgeSpec> {
    let v = k.cross(u);
    if !(r > 0.0 && pitch > 0.0 && (a1 - a0).abs() > 1e-12) {
        return Err(KernelError::Invalid("poc helix parameters".into()));
    }
    let n = (((a1 - a0).abs() / HELIX_PIECE).ceil() as usize).max(1);
    let h = (a1 - a0) / n as f64;
    let kappa = 4.0 / 3.0 * (h / 4.0).tan();
    let at = |a: f64| helix_point(origin, k, u, r, z0, pitch, a);
    let rise = k * (pitch * h / TAU / 3.0);
    let turn = |a: f64| (u * (-a.sin()) + v * a.cos()) * (r * kappa);
    let mut ctrl = vec![at(a0)];
    for i in 0..n {
        let (s, e) = (a0 + h * i as f64, a0 + h * (i + 1) as f64);
        ctrl.extend([at(s) + turn(s) + rise, at(e) - turn(e) - rise, at(e)]);
    }
    Ok(EdgeSpec::Chain { degree: 3, ctrl: ctrl.into() })
}

/// Vertical axis edge as a cubic chain with the same piece count as a helix over `[a0,a1]`.
fn axis_chain(origin: Vec3, k: Vec3, z0_helix: f64, pitch: f64, a0: f64, a1: f64) -> Result<EdgeSpec> {
    let n = (((a1 - a0).abs() / HELIX_PIECE).ceil() as usize).max(1);
    let h = (a1 - a0) / n as f64;
    let z_at = |a: f64| origin + k * (z0_helix + pitch * a / TAU);
    let mut ctrl = vec![z_at(a0)];
    for i in 0..n {
        let (s, e) = (a0 + h * i as f64, a0 + h * (i + 1) as f64);
        let (ps, pe) = (z_at(s), z_at(e));
        ctrl.extend([ps + (pe - ps) / 3.0, ps + (pe - ps) * (2.0 / 3.0), pe]);
    }
    Ok(EdgeSpec::Chain { degree: 3, ctrl: ctrl.into() })
}

/// Plane ∩ meridional flank: samples lie on both the ruled surface and the plane `z`.
pub fn plane_intersect_flank(
    origin: Vec3,
    k: Vec3,
    u: Vec3,
    r0: f64,
    r1: f64,
    z0_0: f64,
    z0_1: f64,
    pitch: f64,
    z: f64,
) -> Result<EdgeSpec> {
    let v = k.cross(u);
    let a0 = angle_at_z(z0_0, pitch, z);
    let a1 = angle_at_z(z0_1, pitch, z);
    let delta_z = z0_1 - z0_0;
    if delta_z.abs() < 1e-14 || (a1 - a0).abs() < 1e-14 {
        return Err(KernelError::Invalid("poc degenerate flank plane curve".into()));
    }
    let n = 8usize;
    let mut pts = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let t = i as f64 / n as f64;
        let a = a0 + (a1 - a0) * t;
        let z_c = z0_0 + pitch * a / TAU;
        let s = ((z - z_c) / delta_z).clamp(0.0, 1.0);
        let r = r0 + (r1 - r0) * s;
        pts.push(origin + k * z + (u * a.cos() + v * a.sin()) * r);
    }
    let mut ctrl = vec![pts[0]];
    for i in 0..n {
        let (p0, p1) = (pts[i], pts[i + 1]);
        ctrl.extend([p0 + (p1 - p0) / 3.0, p0 + (p1 - p0) * (2.0 / 3.0), p1]);
    }
    Ok(EdgeSpec::Chain { degree: 3, ctrl: ctrl.into() })
}

/// Sample radii along a plane∩flank chain (for intrusion checks).
pub fn chain_sample_radii(origin: Vec3, k: Vec3, e: &EdgeSpec, n: usize) -> Result<Vec<f64>> {
    let EdgeSpec::Chain { ctrl, .. } = e else {
        return Err(KernelError::Invalid("poc expected chain".into()));
    };
    if ctrl.len() < 2 || n < 2 {
        return Err(KernelError::Invalid("poc sample count".into()));
    }
    // Sample polyline through Bézier end-points of each cubic piece (every 3rd ctrl after the first).
    let mut ends = vec![ctrl[0]];
    let mut i = 1;
    while i + 2 < ctrl.len() {
        ends.push(ctrl[i + 2]);
        i += 3;
    }
    if ends.last() != ctrl.last() {
        if let Some(p) = ctrl.last() {
            ends.push(*p);
        }
    }
    let mut out = Vec::with_capacity(n);
    for j in 0..n {
        let t = j as f64 / (n - 1) as f64;
        let f = t * (ends.len() - 1) as f64;
        let i0 = (f.floor() as usize).min(ends.len() - 2);
        let u = f - i0 as f64;
        let p = ends[i0] * (1.0 - u) + ends[i0 + 1] * u;
        let axial = (p - origin).dot(k);
        let r = ((p - origin) - k * axial).len();
        out.push(r);
    }
    Ok(out)
}

/// M5×0.8 one-flank PoC: meridional ruled flank + plane∩ shared with planar caps, closed by
/// axis-ruled “pie” faces (crest↔axis, root↔axis).
pub fn m5_one_flank_closed_shell() -> Result<Body> {
    let origin = Vec3::ZERO;
    let k = Vec3::Z;
    let u = Vec3::X;
    let (rc, rm, pitch) = (2.1, 2.5, 0.8);
    let foot = (pitch - pitch / 4.0) / 2.0;
    let crest_h = pitch / 16.0;
    let z_lo = 0.0;
    let z_hi = pitch * 2.0; // short cell
    let z0 = z_lo - pitch;

    // Meridional generators: shared angle range with margin past plane crossings.
    let margin = 0.25;
    let a_lo = angle_at_z(z0 + foot, pitch, z_lo) - margin;
    let a_hi = angle_at_z(z0 - foot, pitch, z_hi) + margin;
    let fl_gen = helix_edge(origin, k, u, rc, z0 - foot, pitch, a_lo, a_hi)?;
    let cl_gen = helix_edge(origin, k, u, rm, z0 - crest_h, pitch, a_lo, a_hi)?;

    let span = TAU * (z_hi - z_lo) / pitch;
    let a_fl_lo = angle_at_z(z0 - foot, pitch, z_lo);
    let a_fl_hi = a_fl_lo + span;
    let a_cl_lo = angle_at_z(z0 - crest_h, pitch, z_lo);
    let a_cl_hi = a_cl_lo + span;

    let fl = helix_edge(origin, k, u, rc, z0 - foot, pitch, a_fl_lo, a_fl_hi)?;
    let cl = helix_edge(origin, k, u, rm, z0 - crest_h, pitch, a_cl_lo, a_cl_hi)?;

    // Shared plane∩ edges (built once). Directions chosen so the flank loop closes:
    // fl → top(crest→root) → rev(cl) → bot(root→crest).
    let bot_cr = plane_intersect_flank(origin, k, u, rc, rm, z0 - foot, z0 - crest_h, pitch, z_lo)?;
    let top_cr = plane_intersect_flank(origin, k, u, rc, rm, z0 - foot, z0 - crest_h, pitch, z_hi)?;
    let bot_rc = rev(&bot_cr);
    let top = top_cr.clone();

    let fl0 = helix_point(origin, k, u, rc, z0 - foot, pitch, a_fl_lo);
    let fl1 = helix_point(origin, k, u, rc, z0 - foot, pitch, a_fl_hi);
    let cl0 = helix_point(origin, k, u, rm, z0 - crest_h, pitch, a_cl_lo);
    let cl1 = helix_point(origin, k, u, rm, z0 - crest_h, pitch, a_cl_hi);
    let o0 = origin + k * z_lo;
    let o1 = origin + k * z_hi;

    let axis_fl = axis_chain(origin, k, z0 - foot, pitch, a_fl_lo, a_fl_hi)?;
    let axis_cl = axis_chain(origin, k, z0 - crest_h, pitch, a_cl_lo, a_cl_hi)?;

    let v = k.cross(u);
    let mid_a = 0.5 * (a_fl_lo + a_cl_lo);
    let mid_z = 0.5 * (z_lo + z_hi);
    let out = |a: f64| u * a.cos() + v * a.sin();
    let at = |r: f64, a: f64, z: f64| origin + k * z + out(a) * r;

    // Planar caps share the same EdgeSpecs as the flank free edges (opposite use via rev in sew).
    let bot_loop = vec![bot_cr.clone(), line(cl0, o0), line(o0, fl0)];
    let top_loop = vec![top_cr.clone(), line(cl1, o1), line(o1, fl1)];

    let faces = [
        FaceSpec {
            loops: vec![vec![fl.clone(), top, rev(&cl), bot_rc]],
            surface: SurfSpec::Ruled { a: fl_gen, b: cl_gen },
            probe: Some((at((rc + rm) / 2.0, mid_a, mid_z), out(mid_a) * 0.5)),
        },
        FaceSpec {
            loops: vec![bot_loop],
            surface: SurfSpec::Plane,
            probe: None,
        },
        FaceSpec {
            loops: vec![top_loop.into_iter().rev().map(|e| rev(&e)).collect()],
            surface: SurfSpec::Plane,
            probe: None,
        },
        FaceSpec {
            loops: vec![vec![fl.clone(), line(fl1, o1), rev(&axis_fl), line(o0, fl0)]],
            surface: SurfSpec::Ruled { a: fl, b: axis_fl.clone() },
            probe: Some((at(rc * 0.5, a_fl_lo + 0.1, mid_z), out(a_fl_lo))),
        },
        FaceSpec {
            loops: vec![vec![cl.clone(), line(cl1, o1), rev(&axis_cl), line(o0, cl0)]],
            surface: SurfSpec::Ruled { a: cl, b: axis_cl.clone() },
            probe: Some((at(rm * 0.5, a_cl_lo + 0.1, mid_z), -out(a_cl_lo))),
        },
    ];

    crate::sew(&faces)
}

/// Measurements for the entry plane∩flank curve alone (no sew).
pub struct IntersectReport {
    pub min_radius: f64,
    pub mid_radius: f64,
    pub max_dev_from_plane: f64,
    pub crest: f64,
}

pub fn measure_m5_entry_intersect() -> Result<IntersectReport> {
    let origin = Vec3::ZERO;
    let k = Vec3::Z;
    let u = Vec3::X;
    let (rc, rm, pitch) = (2.1, 2.5, 0.8);
    let foot = (pitch - pitch / 4.0) / 2.0;
    let crest_h = pitch / 16.0;
    let z_lo = 0.0;
    let z0 = z_lo - pitch;
    let bot = plane_intersect_flank(origin, k, u, rc, rm, z0 - foot, z0 - crest_h, pitch, z_lo)?;
    let radii = chain_sample_radii(origin, k, &bot, 17)?;
    let min_radius = radii.iter().copied().fold(f64::INFINITY, f64::min);
    let mid_radius = radii[radii.len() / 2];
    let EdgeSpec::Chain { ctrl, .. } = &bot else {
        return Err(KernelError::Invalid("poc chain".into()));
    };
    let max_dev_from_plane = ctrl.iter().map(|p| (p.z - z_lo).abs()).fold(0.0, f64::max);
    Ok(IntersectReport { min_radius, mid_radius, max_dev_from_plane, crest: rc })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn within(budget: Duration, start: Instant) {
        assert!(start.elapsed() < budget, "exceeded {:?} (elapsed {:?})", budget, start.elapsed());
    }

    #[test]
    fn plane_intersect_stays_outside_crest_and_on_plane() {
        let t0 = Instant::now();
        let r = measure_m5_entry_intersect().expect("measure");
        eprintln!(
            "intersect: min_r={:.4} mid_r={:.4} plane_dev={:.3e} crest={}",
            r.min_radius, r.mid_radius, r.max_dev_from_plane, r.crest
        );
        assert!(r.max_dev_from_plane < 1e-9, "off plane {}", r.max_dev_from_plane);
        assert!(r.min_radius + 1e-3 >= r.crest, "intrudes: min_r={} crest={}", r.min_radius, r.crest);
        assert!(r.mid_radius + 1e-3 >= r.crest, "mid chord intrusion {}", r.mid_radius);
        // Must not be the old straight-chord mid (~1.29).
        assert!(r.mid_radius > 2.0, "mid_r {} looks like constant-z chord", r.mid_radius);
        within(Duration::from_secs(60), t0);
    }

    #[test]
    fn one_flank_meridional_shell_closes() {
        let t0 = Instant::now();
        let body = m5_one_flank_closed_shell().expect("sew closed shell");
        let sew_elapsed = t0.elapsed();
        eprintln!("sew {:?}", sew_elapsed);
        assert!(sew_elapsed < Duration::from_secs(90), "sew exceeded 90s: {:?}", sew_elapsed);
        assert!(!body.is_surface(), "open surface, not closed");
        assert_eq!(body.shell_count(), 1);
        // Avoid fine validity() meshing hang; closedness is !is_surface.
        let vol = body.tessellate(0.3).expect("coarse tess").measure().volume;
        eprintln!("vol={vol:.4} faces={}", body.face_count());
        assert!(vol > 0.05 && vol < 50.0, "unexpected volume {vol}");
        within(Duration::from_secs(60), t0);
    }
}
