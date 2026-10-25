//! Servo Mount: the case cut-out and two tab holes, measured from the solved sketch.

use serde_json::{Value, json};
use solidcraft_sketch::CurveKind;

use crate::Session;

fn run(s: &mut Session, id: &str, p: Value) -> Value {
    match s.execute(id, &p) {
        Ok(v) => v,
        Err(e) => panic!("{id} {p}: {e}"),
    }
}

fn sketch() -> Session {
    let mut s = Session::default();
    run(&mut s, "sketch.create", json!({"plane": "XY"}));
    s
}

fn solved(s: &Session) -> solidcraft_sketch::Sketch {
    let id = s.active_sketch.expect("an open sketch");
    s.model.state().sketch(id).map(|ss| ss.sketch.clone()).expect("an evaluated sketch")
}

/// A circle's centre and diameter, from the solved sketch.
fn circle(s: &Session, id: &Value) -> ([f64; 2], f64) {
    let sk = solved(s);
    let id = id.as_str().expect("a circle id");
    let i = sk.curve_index(id).expect("circle in sketch");
    match &sk.curves[i].kind {
        CurveKind::Circle { c, r } => {
            let p = sk.point(*c).expect("centre");
            ([p.x, p.y], 2.0 * r)
        }
        k => panic!("{id} is not a circle: {k:?}"),
    }
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-6, "{a} != {b}");
}

/// The areas of the sketch's profiles, largest first.
fn areas(si: &Value) -> Vec<f64> {
    let mut a: Vec<f64> = si["profiles"].as_array().expect("profiles").iter().filter_map(|p| p["area_mm2"].as_f64()).collect();
    a.sort_by(|x, y| y.total_cmp(x));
    a
}

#[test]
fn an_mg90s_mount_on_the_origin_is_fully_constrained() {
    let mut s = sketch();
    let out = run(&mut s, "sketch.servo_mount", json!({}));
    assert_eq!(out["model"], "MG90S");
    let si = run(&mut s, "sketch.inspect", json!({}));
    assert_ne!(si["status"], "conflict", "{si}");
    assert!(si["status"].as_str().is_none_or(|t| !t.contains("over")), "{si}");
    assert_eq!(si["dof"], 0, "the centre is on the origin, everything else is dimensioned: {si}");

    // The case cut-out is the nominal 22.8 × 12.2 plus the 0.2 mm fit.
    let a = areas(&si);
    assert_eq!(a.len(), 3, "a rectangle and two holes: {si}");
    close(a[0], 23.0 * 12.4);
    let hole_area = std::f64::consts::PI * 1.15 * 1.15;
    close(a[1], hole_area);
    close(a[2], hole_area);

    let (c0, d0) = circle(&s, &out["holes"][0]);
    let (c1, d1) = circle(&s, &out["holes"][1]);
    close(d0, 2.3);
    close(d1, 2.3);
    close(c0[0], -13.9);
    close(c1[0], 13.9);
    close(c0[1], 0.0);
    close(c1[1], 0.0);
}

#[test]
fn an_sg90_turned_upright_away_from_the_origin_keeps_only_its_position_free() {
    let mut s = sketch();
    let out = run(&mut s, "sketch.servo_mount", json!({"model": "sg90", "center": [50, 20], "angle": 90, "fit": 0}));
    assert_eq!(out["model"], "SG90");
    let si = run(&mut s, "sketch.inspect", json!({}));
    assert_eq!(si["dof"], 2, "{si}");
    close(areas(&si)[0], 22.2 * 11.8);
    let (c0, _) = circle(&s, &out["holes"][0]);
    let (c1, _) = circle(&s, &out["holes"][1]);
    close(c0[0], 50.0);
    close(c1[0], 50.0);
    close((c1[1] - c0[1]).abs(), 27.5);
    close((c0[1] + c1[1]) / 2.0, 20.0);
}

#[test]
fn a_parameter_drives_the_hole_pitch() {
    let mut s = sketch();
    run(&mut s, "parameters.add", json!({"name": "servo_pitch", "expression": "30 mm"}));
    let out = run(&mut s, "sketch.servo_mount", json!({"pitch": "servo_pitch"}));
    let spread = |s: &Session| {
        let (a, _) = circle(s, &out["holes"][0]);
        let (b, _) = circle(s, &out["holes"][1]);
        b[0] - a[0]
    };
    close(spread(&s), 30.0);
    let changed = run(&mut s, "parameters.change", json!({"name": "servo_pitch", "expression": "32 mm"}));
    assert_eq!(changed["errors"], json!([]));
    close(spread(&s), 32.0);
    let si = run(&mut s, "sketch.inspect", json!({}));
    assert_eq!(si["dof"], 0, "{si}");
}

#[test]
fn the_mount_is_one_undo_step() {
    let mut s = sketch();
    run(&mut s, "sketch.servo_mount", json!({"center": [5, 5]}));
    run(&mut s, "edit.undo", json!({}));
    let sk = solved(&s);
    assert!(sk.curves.is_empty(), "undo removes the whole mount: {:?}", sk.curves.iter().map(|c| &c.id).collect::<Vec<_>>());
}

#[test]
fn bad_input_is_refused_and_draws_nothing() {
    for p in [
        json!({"model": "MG996R"}),
        json!({"angle": 45}),
        json!({"center": "middle"}),
        json!({"pitch": 20}),
        json!({"hole": 0}),
        json!({"length": "nope"}),
    ] {
        let mut s = sketch();
        assert!(s.execute("sketch.servo_mount", &p).is_err(), "{p} accepted");
        assert!(solved(&s).curves.is_empty(), "{p} left curves behind");
    }
    let mut s = Session::default();
    assert!(s.execute("sketch.servo_mount", &json!({})).is_err(), "needs an open sketch");
}
