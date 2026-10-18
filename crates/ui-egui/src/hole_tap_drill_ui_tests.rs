//! Tapped holes in the Hole dialog use the conventional tap-drill diameter, kept separate
//! from the thread's nominal size. An explicit bore edit is preserved when the size changes.

use serde_json::json;
use solidcraft_engine::doc::{FeatureKind, tap_drill_diameter};
use solidcraft_engine::geom::Vec3;

use crate::dialogs::{self, Kind};
use crate::scenario::Harness;

fn plate() -> Harness {
    let mut h = Harness::new();
    h.app.ui.show_origin = false;
    h.app.run("solid.box", json!({"length": 40, "width": 50, "height": 10})).unwrap();
    h.step(&json!({"call": "ui.view", "params": {"view": "top"}})).unwrap();
    h.settle();
    h
}

fn hole_diameter(h: &Harness) -> String {
    let f = h.app.session.doc.features.last().unwrap();
    match &f.kind {
        FeatureKind::Hole { diameter, thread, .. } => {
            assert_eq!(thread.as_deref(), Some("M5"), "{f:?}");
            diameter.to_string()
        }
        other => panic!("expected a hole, got {}", other.type_name()),
    }
}

#[test]
fn tapped_m5_from_the_hole_dialog_drills_4_2_mm() {
    let mut h = plate();
    h.step(&json!({"start": "solid.hole"})).unwrap();
    let d = h.app.dialog.as_mut().unwrap();
    if let Kind::Hole { diameter, opts, .. } = &mut d.kind {
        opts.tapped = true;
        opts.thread = "M5".into();
        opts.tap_diameter_override = false;
        dialogs::sync_tapped_hole_diameter(diameter, &opts.thread);
        assert_eq!(diameter, "4.2 mm", "tap drill for M5 × 0.8");
    } else {
        panic!("hole dialog");
    }
    // Place on the top face centre and OK.
    let at = h.at(&json!({"world": [20.0, 25.0, 10.0]})).unwrap();
    h.step(&json!({"click": at})).unwrap();
    h.step(&json!({"key": "Enter"})).unwrap();
    assert!(h.app.dialog.is_none(), "OK closed: {:?}", h.app.dialog.as_ref().map(|d| &d.error));
    let dia = hole_diameter(&h);
    assert!(dia == "4.2 mm" || dia == "4.2", "stored bore is the tap drill, got {dia}");
    let st = h.app.session.world_state();
    let wall = Vec3::new(20.0 + 2.1, 25.0, 5.0);
    let cyl = st.bodies.iter().find_map(|b| solidcraft_engine::kernel::cylinder_face_at(&b.body, wall));
    let cyl = cyl.expect("the tapped hole leaves a cylinder of the tap-drill radius");
    assert!((cyl.radius * 2.0 - 4.2).abs() < 1e-6, "bore diameter {}", cyl.radius * 2.0);
}

#[test]
fn changing_thread_size_updates_the_tap_drill_unless_overridden() {
    assert!((tap_drill_diameter(5.0, 0.8) - 4.2).abs() < 1e-12);
    assert!((tap_drill_diameter(6.0, 1.0) - 5.0).abs() < 1e-12);
    assert!((tap_drill_diameter(8.0, 1.25) - 6.8).abs() < 1e-12);

    let mut diameter = "4.2 mm".to_string();
    dialogs::sync_tapped_hole_diameter(&mut diameter, "M6");
    assert_eq!(diameter, "5 mm");
    dialogs::sync_tapped_hole_diameter(&mut diameter, "M8");
    assert_eq!(diameter, "6.8 mm");
}
