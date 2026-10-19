//! Plane-trimmed helical thread ends: the groove is a continuous helix clipped by the entry/exit
//! planes. Free boundaries lie in those planes (long crest arc + flank segments + short root arc).
//! There are no radial planar end-caps inside the solid.

use std::f64::consts::{PI, TAU};
use std::sync::Arc;

use solidcraft_geom::Vec3;
use truck_modeling as mt;
use truck_modeling::{BoundedCurve, ParametricCurve};

use crate::body::from_p3;
use crate::sew::{EdgeSpec, FaceSpec, SurfSpec};
use crate::{Body, KernelError, Result};

use crate::helical_thread::{HELIX_PIECE, basic_h, tap_radius};

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

fn helix_point(origin: Vec3, k: Vec3, u: Vec3, r: f64, z0: f64, pitch: f64, a: f64) -> Vec3 {
    let v = k.cross(u);
    origin + k * (z0 + pitch * a / TAU) + (u * a.cos() + v * a.sin()) * r
}

fn angle_at_z(z0: f64, pitch: f64, z: f64) -> f64 {
    TAU * (z - z0) / pitch
}

/// Circular helix edge from angle `a0` to `a1` (pitch may be signed).
fn helix_edge(origin: Vec3, k: Vec3, u: Vec3, r: f64, z0: f64, pitch: f64, a0: f64, a1: f64) -> Result<EdgeSpec> {
    let v = k.cross(u);
    if !(r > 0.0 && pitch.abs() > 0.0 && (a1 - a0).abs() > 1e-12) {
        return Err(KernelError::Invalid("helix parameters".into()));
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

/// Arc(s) on the circle of radius `r` at axial `z`, from angle `a0` to `a1` (signed span).
/// Splits into pieces of at most π so each truck arc has a valid midpoint.
fn circle_arcs(origin: Vec3, k: Vec3, u: Vec3, r: f64, z: f64, a0: f64, a1: f64) -> Result<Vec<EdgeSpec>> {
    let v = k.cross(u);
    let p = |a: f64| origin + k * z + (u * a.cos() + v * a.sin()) * r;
    let mut span = a1 - a0;
    while span > TAU {
        span -= TAU;
    }
    while span < -TAU {
        span += TAU;
    }
    if span.abs() < 1e-12 {
        return Err(KernelError::Invalid("degenerate circle arc".into()));
    }
    let n = ((span.abs() / PI).ceil() as usize).max(1);
    let h = span / n as f64;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let (s, e) = (a0 + h * i as f64, a0 + h * (i + 1) as f64);
        out.push(EdgeSpec::Arc { a: p(s), b: p(e), mid: p(0.5 * (s + e)) });
    }
    Ok(out)
}

/// ISO metric profile offsets. Centreline intercept sits one pitch below `z_lo` so the
/// construction helix extends past both faces; edges are clipped by [`angle_at_z`].
fn profile(pitch: f64) -> (f64, f64) {
    let foot = (pitch - pitch / 4.0) / 2.0;
    let crest = pitch / 16.0;
    (foot, crest)
}

/// Closed planar entry/exit loop at axial `z`: short crest arc + upper flank + short root + lower
/// flank. Helix ends at this plane are coplanar, so the flank edges are the ruled-surface end
/// rulings (straight) and lie on both the wall and the plate face.
pub(crate) fn entry_loop(origin: Vec3, k: Vec3, u: Vec3, rc: f64, rm: f64, z0: f64, pitch: f64, foot: f64, crest: f64, z: f64) -> Result<Vec<EdgeSpec>> {
    let a_fl = angle_at_z(z0 - foot, pitch, z);
    let a_fu = angle_at_z(z0 + foot, pitch, z);
    let a_cl = angle_at_z(z0 - crest, pitch, z);
    let a_cu = angle_at_z(z0 + crest, pitch, z);
    let fl = helix_point(origin, k, u, rc, z0 - foot, pitch, a_fl);
    let fu = helix_point(origin, k, u, rc, z0 + foot, pitch, a_fu);
    let cl = helix_point(origin, k, u, rm, z0 - crest, pitch, a_cl);
    let cu = helix_point(origin, k, u, rm, z0 + crest, pitch, a_cu);
    // Short crest land (¼ turn): the groove occupies the remaining ¾ turn at this z-cut.
    let mut loop_e = circle_arcs(origin, k, u, rc, z, a_fl, a_fu + TAU)?;
    loop_e.push(line(fu, cu));
    loop_e.extend(circle_arcs(origin, k, u, rm, z, a_cu, a_cl)?);
    loop_e.push(line(cl, fl));
    Ok(loop_e)
}

/// Open helical wall whose free boundaries are the planar entry loops at `z_lo` and `z_hi`.
/// No radial end-cap faces — the groove is trimmed by those planes.
pub fn internal_thread_wall_trimmed(
    origin: Vec3,
    axis: Vec3,
    u: Vec3,
    major: f64,
    pitch: f64,
    crest: f64,
    z_lo: f64,
    z_hi: f64,
) -> Result<(Body, Vec<EdgeSpec>, Vec<EdgeSpec>)> {
    let k = axis.normalized().ok_or_else(|| KernelError::Invalid("thread axis".into()))?;
    let u = u.normalized().ok_or_else(|| KernelError::Invalid("thread radial".into()))?;
    let rc = crest;
    let rm = major / 2.0;
    let height = z_hi - z_lo;
    if !(height > pitch * 2.0 && rm > rc && rc > 0.0 && pitch > 0.0) {
        return Err(KernelError::Invalid("thread height / diameter".into()));
    }
    let h = basic_h(pitch);
    if rm - rc < (5.0 * h / 8.0) / 4.0 {
        return Err(KernelError::Invalid("thread profile too shallow".into()));
    }
    let (foot, crest_h) = profile(pitch);
    // Centreline one pitch below the lower face so the mathematical helix extends past both planes.
    let z0 = z_lo - pitch;
    // One shared angular span for every helix so ruled flank chains keep identical piece counts
    // (per-helix angle_at_z(z_hi) can differ by an ulp after boolean cylinder extents).
    let span = TAU * height / pitch;
    let a_fl_lo = angle_at_z(z0 - foot, pitch, z_lo);
    let a_fl_hi = a_fl_lo + span;
    let a_fu_lo = angle_at_z(z0 + foot, pitch, z_lo);
    let a_fu_hi = a_fu_lo + span;
    let a_cl_lo = angle_at_z(z0 - crest_h, pitch, z_lo);
    let a_cl_hi = a_cl_lo + span;
    let a_cu_lo = angle_at_z(z0 + crest_h, pitch, z_lo);
    let a_cu_hi = a_cu_lo + span;

    let fl = helix_edge(origin, k, u, rc, z0 - foot, pitch, a_fl_lo, a_fl_hi)?;
    let fu = helix_edge(origin, k, u, rc, z0 + foot, pitch, a_fu_lo, a_fu_hi)?;
    let cl = helix_edge(origin, k, u, rm, z0 - crest_h, pitch, a_cl_lo, a_cl_hi)?;
    let cu = helix_edge(origin, k, u, rm, z0 + crest_h, pitch, a_cu_lo, a_cu_hi)?;
    let (fl0, fl1) = ends(&fl)?;
    let (fu0, fu1) = ends(&fu)?;
    let (cl0, cl1) = ends(&cl)?;
    let (cu0, cu1) = ends(&cu)?;

    let bot_loop = entry_loop(origin, k, u, rc, rm, z0, pitch, foot, crest_h, z_lo)?;
    let top_loop = entry_loop(origin, k, u, rc, rm, z0, pitch, foot, crest_h, z_hi)?;

    let bot_crest = circle_arcs(origin, k, u, rc, z_lo, a_fl_lo, a_fu_lo + TAU)?;
    let top_crest = circle_arcs(origin, k, u, rc, z_hi, a_fl_hi, a_fu_hi + TAU)?;
    let bot_root = circle_arcs(origin, k, u, rm, z_lo, a_cu_lo, a_cl_lo)?;
    let top_root = circle_arcs(origin, k, u, rm, z_hi, a_cu_hi, a_cl_hi)?;

    let v = k.cross(u);
    let at = |r: f64, a: f64, z: f64| origin + k * z + (u * a.cos() + v * a.sin()) * r;
    let out = |a: f64| u * a.cos() + v * a.sin();
    let mid_z = 0.5 * (z_lo + z_hi);
    let mid_a = angle_at_z(z0, pitch, mid_z);
    let flank = 0.5_f64;

    let mut crest_loop: Vec<EdgeSpec> = bot_crest;
    crest_loop.push(fu.clone());
    crest_loop.extend(top_crest.iter().rev().map(rev));
    crest_loop.push(rev(&fl));

    let mut root_loop: Vec<EdgeSpec> = bot_root;
    root_loop.push(cl.clone());
    root_loop.extend(top_root.iter().rev().map(rev));
    root_loop.push(rev(&cu));

    let wall = crate::sew_open(&[
        FaceSpec {
            loops: vec![crest_loop],
            surface: SurfSpec::Revolved {
                a: origin + u * rc + k * z_lo,
                b: origin + u * rc + k * z_hi,
                axis_origin: origin,
                axis: k,
                angle: TAU,
            },
            probe: Some((at(rc, mid_a + PI, mid_z), -out(mid_a + PI))),
        },
        FaceSpec {
            loops: vec![vec![fl.clone(), line(fl1, cl1), rev(&cl), line(cl0, fl0)]],
            surface: SurfSpec::Ruled { a: fl.clone(), b: cl.clone() },
            probe: Some((at((rc + rm) / 2.0, mid_a, mid_z), out(mid_a) * flank)),
        },
        FaceSpec {
            loops: vec![vec![cu.clone(), line(cu1, fu1), rev(&fu), line(fu0, cu0)]],
            surface: SurfSpec::Ruled { a: fu.clone(), b: cu.clone() },
            probe: Some((at((rc + rm) / 2.0, mid_a, mid_z), out(mid_a) * flank)),
        },
        FaceSpec {
            loops: vec![root_loop],
            surface: SurfSpec::Revolved {
                a: origin + u * rm + k * z_lo,
                b: origin + u * rm + k * z_hi,
                axis_origin: origin,
                axis: k,
                angle: TAU,
            },
            probe: Some((at(rm, mid_a, mid_z), out(mid_a))),
        },
    ])?;

    Ok((wall, bot_loop, top_loop))
}

fn truck_edge_to_spec(e: &mt::Edge) -> Result<EdgeSpec> {
    let abs = if e.orientation() { e.clone() } else { e.inverse() };
    let c = abs.curve();
    let (t0, t1) = c.range_tuple();
    if !(t0.is_finite() && t1.is_finite()) {
        return Err(KernelError::Failed("non-finite edge parameter range".into()));
    }
    let a = from_p3(c.subs(t0));
    let b = from_p3(c.subs(t1));
    let mid = from_p3(c.subs(0.5 * (t0 + t1)));
    match c {
        mt::Curve::Line(_) => Ok(EdgeSpec::Line { a, b }),
        mt::Curve::BSplineCurve(ref bsp) => {
            let ctrl: Arc<[Vec3]> = bsp.control_points().iter().map(|p| from_p3(*p)).collect::<Vec<_>>().into();
            let degree = bsp.degree();
            if (1..=crate::sew::MAX_CHAIN_DEGREE).contains(&degree) && ctrl.len() > degree && (ctrl.len() - 1) % degree == 0 {
                Ok(EdgeSpec::Chain { degree, ctrl })
            } else {
                // Fall back to a circular arc through the midpoint (boolean hole rims).
                Ok(EdgeSpec::Arc { a, b, mid })
            }
        }
        _ => Ok(EdgeSpec::Arc { a, b, mid }),
    }
}

fn wire_to_loop(w: &mt::Wire) -> Result<Vec<EdgeSpec>> {
    let mut out = Vec::new();
    for e in w.edge_iter() {
        out.push(truck_edge_to_spec(&e)?);
    }
    if out.is_empty() {
        return Err(KernelError::Failed("empty face wire".into()));
    }
    weld_loop_ends(&mut out);
    Ok(out)
}

/// Force consecutive EdgeSpec ends to share identical coordinates so sew sees a closed wire
/// after round-tripping truck curves through Arc/Line approximations.
fn weld_loop_ends(edges: &mut [EdgeSpec]) {
    if edges.len() < 2 {
        return;
    }
    let snap = |e: &mut EdgeSpec, at_start: bool, p: Vec3| match e {
        EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => {
            if at_start {
                *a = p;
            } else {
                *b = p;
            }
        }
        EdgeSpec::Chain { ctrl, .. } => {
            let mut v = ctrl.to_vec();
            if at_start {
                if let Some(x) = v.first_mut() {
                    *x = p;
                }
            } else if let Some(x) = v.last_mut() {
                *x = p;
            }
            *ctrl = v.into();
        }
    };
    let end_of = |e: &EdgeSpec| match e {
        EdgeSpec::Line { b, .. } | EdgeSpec::Arc { b, .. } => *b,
        EdgeSpec::Chain { ctrl, .. } => ctrl.last().copied().unwrap_or(Vec3::ZERO),
    };
    for i in 0..edges.len() {
        let j = (i + 1) % edges.len();
        let p = end_of(&edges[i]);
        snap(&mut edges[j], true, p);
    }
}

/// Mean of edge endpoints — used to recognise which hole wire belongs to this bore.
#[allow(dead_code)]
fn loop_centroid(edges: &[EdgeSpec]) -> Vec3 {
    let mut s = Vec3::ZERO;
    let mut n = 0.0;
    for e in edges {
        let (a, b) = match e {
            EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
            EdgeSpec::Chain { ctrl, .. } => match (ctrl.first(), ctrl.last()) {
                (Some(a), Some(b)) => (*a, *b),
                _ => continue,
            },
        };
        s = s + a + b;
        n += 2.0;
    }
    if n > 0.0 { s * (1.0 / n) } else { Vec3::ZERO }
}

/// True when every sample of the loop lies near the cylinder of radius `r` about `origin`/`k`.
#[allow(dead_code)]
fn loop_on_cylinder(edges: &[EdgeSpec], origin: Vec3, k: Vec3, r: f64, tol: f64) -> bool {
    let mut samples = 0usize;
    for e in edges {
        let pts: Vec<Vec3> = match e {
            EdgeSpec::Line { a, b } => vec![*a, *b],
            EdgeSpec::Arc { a, b, mid } => vec![*a, *mid, *b],
            EdgeSpec::Chain { ctrl, .. } => ctrl.iter().copied().collect(),
        };
        for p in pts {
            let along = (p - origin).dot(k);
            let radial = ((p - origin) - k * along).len();
            if (radial - r).abs() > tol {
                return false;
            }
            samples += 1;
        }
    }
    samples >= 3
}

/// All closed free-edge loops lying in the plane `(p - origin)·k = z`.
/// Used after `remove_faces` so rebuilt caps rematch sides, plain holes, and prior thread walls exactly.
#[allow(dead_code)]
pub(crate) fn free_loops_on_plane(body: &Body, origin: Vec3, k: Vec3, z: f64, tol: f64) -> Result<Vec<Vec<EdgeSpec>>> {
    let k = k.normalized().ok_or_else(|| KernelError::Invalid("plane axis".into()))?;
    let axial = |p: Vec3| (p - origin).dot(k);
    let mut edges: Vec<EdgeSpec> = Vec::new();
    for e in free_truck_edges(body) {
        let spec = truck_edge_to_spec(&e)?;
        let (a, b) = match &spec {
            EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
            EdgeSpec::Chain { ctrl, .. } => match (ctrl.first(), ctrl.last()) {
                (Some(a), Some(b)) => (*a, *b),
                _ => continue,
            },
        };
        let mid = match &spec {
            EdgeSpec::Arc { mid, .. } => *mid,
            EdgeSpec::Line { a, b } => (*a + *b) * 0.5,
            EdgeSpec::Chain { ctrl, .. } => ctrl.get(ctrl.len() / 2).copied().unwrap_or(a),
        };
        if (axial(a) - z).abs() <= tol && (axial(b) - z).abs() <= tol && (axial(mid) - z).abs() <= tol {
            edges.push(spec);
        }
    }
    if edges.is_empty() {
        return Err(KernelError::Failed("no free edges on the cap plane".into()));
    }
    // Greedy assemble into closed loops.
    let mut loops = Vec::new();
    while !edges.is_empty() {
        let mut loop_e = vec![edges.swap_remove(0)];
        let start = match &loop_e[0] {
            EdgeSpec::Line { a, .. } | EdgeSpec::Arc { a, .. } => *a,
            EdgeSpec::Chain { ctrl, .. } => ctrl.first().copied().unwrap_or(Vec3::ZERO),
        };
        let mut cur = match &loop_e[0] {
            EdgeSpec::Line { b, .. } | EdgeSpec::Arc { b, .. } => *b,
            EdgeSpec::Chain { ctrl, .. } => ctrl.last().copied().unwrap_or(Vec3::ZERO),
        };
        let mut guard = 0usize;
        while cur.dist(start) > tol {
            guard += 1;
            if guard > 10_000 {
                return Err(KernelError::Failed("free plane loop assembly runaway".into()));
            }
            let mut found = None;
            for (i, e) in edges.iter().enumerate() {
                let (a, b) = match e {
                    EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
                    EdgeSpec::Chain { ctrl, .. } => match (ctrl.first(), ctrl.last()) {
                        (Some(a), Some(b)) => (*a, *b),
                        _ => continue,
                    },
                };
                if a.dist(cur) <= tol {
                    found = Some((i, e.clone(), b));
                    break;
                }
                if b.dist(cur) <= tol {
                    found = Some((i, rev(e), a));
                    break;
                }
            }
            let Some((i, edge, next)) = found else {
                return Err(KernelError::Failed("free edges on the cap plane do not form closed loops".into()));
            };
            edges.swap_remove(i);
            loop_e.push(edge);
            cur = next;
        }
        weld_loop_ends(&mut loop_e);
        loops.push(loop_e);
    }
    Ok(loops)
}

fn free_truck_edges(body: &Body) -> Vec<mt::Edge> {
    let mut uses: std::collections::HashMap<mt::EdgeID, (usize, mt::Edge)> = std::collections::HashMap::new();
    for f in body.solid.face_iter() {
        for e in f.boundaries().iter().flat_map(|w| w.edge_iter()) {
            uses.entry(e.id()).or_insert((0, e.clone())).0 += 1;
        }
    }
    uses.into_values().filter(|(n, _)| *n == 1).map(|(_, e)| e).collect()
}

/// Planar face indices at `z_lo` / `z_hi` whose boundaries include this bore's circular rim.
pub(crate) fn cap_faces_for_bore(body: &Body, origin: Vec3, k: Vec3, r: f64, z_lo: f64, z_hi: f64, tol: f64) -> Result<(usize, usize)> {
    let mut bot = None;
    let mut top = None;
    for i in 0..body.face_count() {
        let Some(crate::FaceSurface::Plane { point, normal }) = crate::face_surface(body, i) else {
            continue;
        };
        let Some(n) = normal.normalized() else { continue };
        if n.cross(k).len() > 1e-6 {
            continue;
        }
        let z = (point - origin).dot(k);
        let face = body.solid.face_iter().nth(i).ok_or_else(|| KernelError::Failed("face index".into()))?;
        let has_rim = face.absolute_boundaries().iter().any(|w| {
            wire_to_loop(w).ok().is_some_and(|loop_e| loop_on_cylinder(&loop_e, origin, k, r, tol))
        });
        if !has_rim {
            continue;
        }
        if (z - z_lo).abs() <= tol {
            bot = Some(i);
        } else if (z - z_hi).abs() <= tol {
            top = Some(i);
        }
    }
    match (bot, top) {
        (Some(b), Some(t)) => Ok((b, t)),
        _ => Err(KernelError::Failed("could not find planar entry/exit faces for the threaded bore".into())),
    }
}

/// All boundary loops of a planar face as EdgeSpecs, in absolute-boundary order.
#[allow(dead_code)]
pub(crate) fn planar_face_loops(body: &Body, face: usize) -> Result<Vec<Vec<EdgeSpec>>> {
    let f = body.solid.face_iter().nth(face).ok_or_else(|| KernelError::Failed("face index".into()))?;
    f.absolute_boundaries().iter().map(wire_to_loop).collect()
}

/// Rebuild a planar cap: replace the bore's circular (or prior) hole wire with `entry`, keep the rest.
#[allow(dead_code)]
pub(crate) fn rebuild_cap_with_entry(
    loops: Vec<Vec<EdgeSpec>>,
    origin: Vec3,
    k: Vec3,
    old_r: f64,
    entry: Vec<EdgeSpec>,
    tol: f64,
) -> Result<Body> {
    if loops.is_empty() {
        return Err(KernelError::Failed("planar face has no loops".into()));
    }
    let mut replaced = false;
    let mut out = Vec::with_capacity(loops.len());
    for loop_e in loops {
        let c = loop_centroid(&loop_e);
        let radial = ((c - origin) - k * (c - origin).dot(k)).len();
        // The bore hole wire is near the axis (centroid ~ origin) and on/near the old rim radius.
        let is_bore = radial <= tol * 5.0 && (loop_on_cylinder(&loop_e, origin, k, old_r, tol) || loop_near_hole(&loop_e, origin, k, old_r, major_hint(old_r), tol));
        if is_bore && !replaced {
            out.push(entry.clone());
            replaced = true;
        } else {
            out.push(loop_e);
        }
    }
    if !replaced {
        return Err(KernelError::Failed("could not identify the bore hole wire on the planar face".into()));
    }
    sew_cap_from_loops(out, Vec::new())
}

/// Sew a planar cap from existing free loops plus a new thread entry (empty `entry` = loops only).
#[allow(dead_code)]
pub(crate) fn sew_cap_from_loops(mut loops: Vec<Vec<EdgeSpec>>, entry: Vec<EdgeSpec>) -> Result<Body> {
    if !entry.is_empty() {
        loops.push(entry);
    }
    if loops.is_empty() {
        return Err(KernelError::Failed("planar face has no loops".into()));
    }
    // Largest loop by approximate perimeter is the outer boundary; sew expects it first.
    loops.sort_by(|a, b| loop_perimeter(b).partial_cmp(&loop_perimeter(a)).unwrap_or(std::cmp::Ordering::Equal));
    for loop_e in &mut loops {
        weld_loop_ends(loop_e);
    }
    crate::sew_open(&[FaceSpec { loops, surface: SurfSpec::Plane, probe: None }])
}

#[allow(dead_code)]
fn loop_perimeter(edges: &[EdgeSpec]) -> f64 {
    edges.iter().fold(0.0, |s, e| {
        let (a, b) = match e {
            EdgeSpec::Line { a, b } | EdgeSpec::Arc { a, b, .. } => (*a, *b),
            EdgeSpec::Chain { ctrl, .. } => match (ctrl.first(), ctrl.last()) {
                (Some(a), Some(b)) => (*a, *b),
                _ => return s,
            },
        };
        s + a.dist(b)
    })
}

/// Replace one cylindrical hole wire on an existing planar face with a new entry loop, keeping
/// every other wire as the original truck edges so stitch rematches prior thread walls.
pub(crate) fn replace_bore_wire_on_face(
    face: &mt::Face,
    origin: Vec3,
    k: Vec3,
    old_r: f64,
    entry: Vec<EdgeSpec>,
    reverse_entry: bool,
    tol: f64,
) -> Result<Body> {
    let entry = if reverse_entry { entry.iter().rev().map(rev).collect() } else { entry };
    let entry_face = crate::sew_open(&[FaceSpec {
        loops: vec![entry],
        surface: SurfSpec::Plane,
        probe: None,
    }])?;
    let entry_wire = entry_face
        .solid
        .face_iter()
        .next()
        .and_then(|f| f.absolute_boundaries().first().cloned())
        .ok_or_else(|| KernelError::Failed("entry face has no wire".into()))?;
    let mut wires = face.absolute_boundaries().clone();
    let mut replaced = false;
    for w in &mut wires {
        let Ok(loop_e) = wire_to_loop(w) else { continue };
        if loop_on_cylinder(&loop_e, origin, k, old_r, tol) || loop_near_hole(&loop_e, origin, k, old_r, major_hint(old_r), tol) {
            let c = loop_centroid(&loop_e);
            let radial = ((c - origin) - k * (c - origin).dot(k)).len();
            if radial <= tol * 5.0 {
                *w = entry_wire.clone();
                replaced = true;
                break;
            }
        }
    }
    if !replaced {
        return Err(KernelError::Failed("could not identify the bore wire on the planar face".into()));
    }
    let mut nf = mt::Face::try_new(wires, face.surface()).map_err(|e| KernelError::Failed(format!("rebuild cap face: {e}")))?;
    if !face.orientation() {
        nf.invert();
    }
    Ok(crate::surfaces::surface_body(vec![nf]))
}

#[allow(dead_code)]
fn major_hint(crest_r: f64) -> f64 {
    // M5 tap 2.1 → major 2.5; allow a band out to ~major for already-notched entry loops.
    crest_r * 1.25
}

#[allow(dead_code)]
fn loop_near_hole(edges: &[EdgeSpec], origin: Vec3, k: Vec3, r_min: f64, r_max: f64, tol: f64) -> bool {
    let c = loop_centroid(edges);
    let axial = (c - origin).dot(k);
    let radial = ((c - origin) - k * axial).len();
    if radial > tol * 5.0 {
        return false;
    }
    edges.iter().all(|e| {
        let pts: Vec<Vec3> = match e {
            EdgeSpec::Line { a, b } => vec![*a, *b],
            EdgeSpec::Arc { a, b, mid } => vec![*a, *mid, *b],
            EdgeSpec::Chain { ctrl, .. } => ctrl.iter().copied().collect(),
        };
        pts.iter().all(|p| {
            let along = (*p - origin).dot(k);
            let rad = ((*p - origin) - k * along).len();
            rad >= r_min - tol && rad <= r_max + tol
        })
    })
}

/// Build a rectangular plate with a plain hole and a plane-trimmed internal helical thread (PoC).
pub fn plate_with_trimmed_internal_thread(
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
    let k = Vec3::Z;
    let u = Vec3::X;
    let origin_t = Vec3::new(thread_xy.0, thread_xy.1, 0.0);
    let origin_p = Vec3::new(plain_xy.0, plain_xy.1, 0.0);
    let (wall, bot_entry, top_entry) = internal_thread_wall_trimmed(origin_t, k, u, major, pitch, rc, 0.0, height)?;

    fn rect_z(z: f64, length: f64, width: f64) -> Vec<EdgeSpec> {
        let (a, b, c, d) = (Vec3::new(0.0, 0.0, z), Vec3::new(length, 0.0, z), Vec3::new(length, width, z), Vec3::new(0.0, width, z));
        vec![line(a, b), line(b, c), line(c, d), line(d, a)]
    }
    fn circle(origin: Vec3, k: Vec3, u: Vec3, r: f64, z: f64) -> Result<Vec<EdgeSpec>> {
        circle_arcs(origin, k, u, r, z, 0.3, 0.3 + TAU)
    }

    let (l, w, hz) = (length, width, height);
    let plain_bot = circle(origin_p, k, u, rc, 0.0)?;
    let plain_top = circle(origin_p, k, u, rc, hz)?;

    let bottom = crate::sew_open(&[FaceSpec {
        loops: vec![
            rect_z(0.0, l, w).into_iter().rev().map(|e| rev(&e)).collect(),
            plain_bot.clone(),
            bot_entry,
        ],
        surface: SurfSpec::Plane,
        probe: None,
    }])?;
    let top = crate::sew_open(&[FaceSpec {
        loops: vec![
            rect_z(hz, l, w),
            plain_top.clone().into_iter().rev().map(|e| rev(&e)).collect(),
            top_entry.into_iter().rev().map(|e| rev(&e)).collect(),
        ],
        surface: SurfSpec::Plane,
        probe: None,
    }])?;

    let sides = crate::sew_open(&[
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
        FaceSpec {
            loops: vec![plain_bot.into_iter().rev().map(|e| rev(&e)).collect(), plain_top],
            surface: SurfSpec::Revolved {
                a: origin_p + u * rc - k,
                b: origin_p + u * rc + k * (hz + 1.0),
                axis_origin: origin_p,
                axis: k,
                angle: TAU,
            },
            probe: Some((origin_p + Vec3::new(-rc, 0.0, hz * 0.5), Vec3::new(-1.0, 0.0, 0.0))),
        },
    ])?;

    match crate::stitch(&[&sides, &wall, &bottom, &top], pitch * 1e-4) {
        Ok(b) if b.validity().is_empty() && b.shell_count() == 1 && !b.is_surface() => Ok(b),
        Ok(b) => Err(KernelError::Failed(format!(
            "trimmed thread plate stitch: shells={}, surface={}, validity={:?}",
            b.shell_count(),
            b.is_surface(),
            b.validity()
        ))),
        Err(e) => Err(KernelError::Failed(format!("trimmed thread plate stitch failed: {e}"))),
    }
}
