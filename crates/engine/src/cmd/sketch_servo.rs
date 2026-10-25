//! Servo Mount: the cut-out for a hobby micro servo's case and its two tab screw holes, drawn
//! into the open sketch as one rigid, dimensioned group in one undo step.
//!
//! The group is the case rectangle (with its centre point), a construction axis through the
//! two hole centres, and the two holes. The rectangle's centre is the axis midpoint, the axis is
//! horizontal (or vertical) and dimensioned to the hole pitch, the holes are equal and share one
//! diameter dimension: only the centre point is free, and a centre on the origin fixes it.

use serde_json::{Value, json};
use solidcraft_doc::expr::Kind;
use solidcraft_geom::Vec2;

use super::{CommandSpec, in_sketch};
use crate::{
    Result, Session,
    params::{bad, expr, num, vec2},
};

pub static COMMANDS: &[CommandSpec] = &[CommandSpec::new("sketch.servo_mount", "Servo Mount", servo_mount)
    .at("SKETCH", "CREATE")
    .icon("rect_center")
    .enabled(in_sketch)
    .params(concat!(
        "model?: SG90|MG90S (default MG90S); center?: [x,y] (default [0,0]; on the origin it is fixed there); angle?: 0|90 (default 0: the case's long side along the sketch x axis); ",
        "fit?: length expr added to the case length and width (default 0.2 mm); length?, width?, pitch?, hole?: length exprs that replace the model's case length, case width, hole pitch and hole diameter ",
        "(nominal values, measure your servo: SG90 22.2 × 11.8, pitch 27.5; MG90S 22.8 × 12.2, pitch 27.8; hole 2.3 for both). ",
        "Draws the case cut-out (a centre rectangle), a construction axis through the hole centres and the two holes, all dimensioned, in one undo step. ",
        "Returns rectangle {sides, corners}, center (point id), axis (line id), holes [2 circle ids], dimensions {length, width, pitch, hole}"
    ))];

/// Nominal sizes of the case and tab holes (mm). Hobby servos vary by maker: these are starting
/// values, and every one can be replaced by a parameter.
struct Servo {
    length: f64,
    width: f64,
    pitch: f64,
    hole: f64,
}

fn model(name: &str) -> Option<Servo> {
    match name.trim().to_ascii_uppercase().as_str() {
        "SG90" => Some(Servo { length: 22.2, width: 11.8, pitch: 27.5, hole: 2.3 }),
        "MG90S" | "MG90" => Some(Servo { length: 22.8, width: 12.2, pitch: 27.8, hole: 2.3 }),
        _ => None,
    }
}

/// A length parameter: the given expression, else the model's value in mm.
fn length(s: &Session, p: &Value, key: &str, default: f64, cmd: &str) -> Result<(String, f64)> {
    let e = expr(p, key).map_or_else(|| format!("{default} mm"), |e| e.to_string());
    let v = s.doc.eval(&e, Kind::Length)?;
    if !v.is_finite() || v.abs() > 1e6 {
        return Err(bad(cmd, format!("`{key}` must be a length")));
    }
    Ok((e, v))
}

fn id_of(v: &Value, cmd: &str, what: &str) -> Result<String> {
    v.as_str().map(str::to_string).ok_or_else(|| bad(cmd, format!("the sketch did not return the {what}")))
}

fn servo_mount(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "sketch.servo_mount";
    let name = p.get("model").and_then(Value::as_str).unwrap_or("MG90S");
    let servo = model(name).ok_or_else(|| bad(cmd, format!("unknown servo model `{name}` (SG90 or MG90S)")))?;
    let center = match p.get("center") {
        None => Vec2::new(0.0, 0.0),
        Some(v) => vec2(v).ok_or_else(|| bad(cmd, "`center` must be a point [x, y]"))?,
    };
    let vertical = match num(p, "angle") {
        None => false,
        Some(a) if a.abs() < 1e-9 => false,
        Some(a) if (a - 90.0).abs() < 1e-9 => true,
        Some(_) => return Err(bad(cmd, "`angle` must be 0 or 90")),
    };
    let (fit_e, fit) = length(s, p, "fit", 0.2, cmd)?;
    let (length_e, len) = length(s, p, "length", servo.length, cmd)?;
    let (width_e, wid) = length(s, p, "width", servo.width, cmd)?;
    let (pitch_e, pitch) = length(s, p, "pitch", servo.pitch, cmd)?;
    let (hole_e, hole) = length(s, p, "hole", servo.hole, cmd)?;
    let (cut_len, cut_wid) = (len + fit, wid + fit);
    if cut_len <= 1e-6 || cut_wid <= 1e-6 || hole <= 1e-6 {
        return Err(bad(cmd, "the case size and the hole must be greater than zero"));
    }
    if pitch - hole <= cut_len {
        return Err(bad(cmd, format!("the holes ({pitch} mm apart, {hole} mm across) would cut into the case cut-out ({cut_len} mm long)")));
    }
    // Sizes as dimension values: a bare model value, or the given expression, plus the fit.
    let sized = |e: &str| if fit.abs() < 1e-12 { e.to_string() } else { format!("({e}) + ({fit_e})") };

    // Along the case's long side, and across it.
    let (along, across) = if vertical { (Vec2::new(0.0, 1.0), Vec2::new(1.0, 0.0)) } else { (Vec2::new(1.0, 0.0), Vec2::new(0.0, 1.0)) };
    let corner = center + along * (cut_len / 2.0) + across * (cut_wid / 2.0);
    let (h0, h1) = (center - along * (pitch / 2.0), center + along * (pitch / 2.0));
    let xy = |v: Vec2| json!([v.x, v.y]);
    let connect = p.get("connect").and_then(Value::as_bool).unwrap_or(true);

    let mut scratch = s.scratch();
    // The centre first, so the rectangle's own centre lands on it and is joined to it.
    let point = scratch.execute("sketch.point", &json!({"point": xy(center), "connect": connect}))?;
    let center_id = id_of(&point["point"], cmd, "centre point")?;
    let rect = scratch.execute("sketch.rectangle.center", &json!({"center": xy(center), "corner": xy(corner)}))?;
    let axis_out = scratch.execute("sketch.line", &json!({"points": [xy(h0), xy(h1)], "construction": true, "infer": true, "connect": false}))?;
    let axis = id_of(&axis_out["curves"][0], cmd, "hole axis")?;
    scratch.execute("sketch.constraint.midpoint", &json!({"point": center_id, "line": axis}))?;
    let mut holes = Vec::new();
    for end in ["start", "end"] {
        let c = scratch.execute("sketch.circle.center", &json!({"center": format!("{axis}.{end}"), "diameter": hole}))?;
        holes.push(id_of(&c["curves"][0], cmd, "hole")?);
    }
    scratch.execute("sketch.constraint.equal", &json!({"a": holes[0], "b": holes[1]}))?;

    let side = |k: &str| id_of(&rect["sides"][k], cmd, "rectangle sides");
    let (long_side, short_side) = if vertical { (side("left")?, side("bottom")?) } else { (side("bottom")?, side("left")?) };
    let mut dims = serde_json::Map::new();
    for (role, entity, ty, value) in [
        ("length", long_side, "length", sized(&length_e)),
        ("width", short_side, "length", sized(&width_e)),
        ("pitch", axis.clone(), "length", pitch_e),
        ("hole", holes[0].clone(), "diameter", hole_e),
    ] {
        let d = scratch.execute("sketch.dimension", &json!({"entities": [entity], "type": ty, "value": value}))?;
        dims.insert(role.into(), d);
    }
    // Session::execute recorded each nested step: this command is the one undo step.
    scratch.undo.clear();
    s.accept_computed(scratch);
    Ok(json!({
        "model": name.trim().to_ascii_uppercase(),
        "rectangle": {"sides": rect["sides"], "corners": rect["corners"]},
        "center": center_id,
        "axis": axis,
        "holes": holes,
        "dimensions": dims,
    }))
}

#[cfg(test)]
#[path = "sketch_servo_tests.rs"]
mod tests;
