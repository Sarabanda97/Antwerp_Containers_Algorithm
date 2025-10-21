use crate::model::{Instance, Demand};

fn face_dir_name(dx: i32, dy: i32) -> Option<&'static str> {
    if dy > 0 { Some("up") }
    else if dy < 0 { Some("down") }
    else if dx > 0 { Some("right") }
    else if dx < 0 { Some("left") }
    else { None }
}

fn emit_move_axis(out: &mut Vec<String>, t: &mut i32, cid: i32, delta: i32, axis: &str) {
    if delta == 0 { return; }
    let dir = if axis == "x" {
        if delta > 0 { "right" } else { "left" }
    } else {
        if delta > 0 { "up" } else { "down" }
    };
    // face (1)
    out.push(format!("{} {} face {}", *t, cid, dir));
    *t += 1;
    // move |delta| (|k|)
    let k = delta.abs();
    out.push(format!("{} {} move {}", *t, cid, k));
    *t += k;
}

pub fn plan_sequential(inst: &Instance) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    if inst.carriers.is_empty() { return out; }
    let cid = inst.carriers[0].id;
    let mut t: i32 = 0;

    // current bottom-left of carrier
    let mut cur = inst.carriers[0].bl;

    for d in &inst.demands {
        match d {
            Demand::Unload { crane_id, container_id: _, storage_id } => {
                // go to crane dispatch (use first dispatch)
                let dispatch = inst.cranes.iter()
                    .find(|c| c.id == *crane_id)
                    .and_then(|c| c.dispatch_positions.get(0).cloned())
                    .unwrap_or(cur);
                // move in y then x
                let dy = dispatch.1 - cur.1;
                emit_move_axis(&mut out, &mut t, cid, dy, "y");
                let dx = dispatch.0 - if dy==0 { cur.0 } else { cur.0 }; // cur updated after moves below
                emit_move_axis(&mut out, &mut t, cid, dx, "x");
                cur = dispatch;

                // load (1)
                out.push(format!("{} {} load", t, cid)); t += 1;

                // go to storage
                let storage_pos = inst.storages.iter()
                    .find(|s| s.id == *storage_id)
                    .map(|s| s.bl)
                    .unwrap_or(cur);
                let dy2 = storage_pos.1 - cur.1;
                emit_move_axis(&mut out, &mut t, cid, dy2, "y");
                let dx2 = storage_pos.0 - cur.0;
                emit_move_axis(&mut out, &mut t, cid, dx2, "x");
                cur = storage_pos;

                // unload
                out.push(format!("{} {} unload", t, cid)); t += 1;
            }
            Demand::Load { crane_id, container_id } => {
                // find container initial storage (fallback first)
                let storage_id_opt = inst.containers_init.iter()
                    .find(|c| c.0 == *container_id)
                    .map(|c| c.1);
                let storage_pos = storage_id_opt
                    .and_then(|sid| inst.storages.iter().find(|s| s.id == sid).map(|s| s.bl))
                    .unwrap_or_else(|| inst.storages.get(0).map(|s| s.bl).unwrap_or(cur));

                // go to storage
                let dy = storage_pos.1 - cur.1;
                emit_move_axis(&mut out, &mut t, cid, dy, "y");
                let dx = storage_pos.0 - cur.0;
                emit_move_axis(&mut out, &mut t, cid, dx, "x");
                cur = storage_pos;

                // load
                out.push(format!("{} {} load", t, cid)); t += 1;

                // go to crane dispatch
                let dispatch = inst.cranes.iter()
                    .find(|c| c.id == *crane_id)
                    .and_then(|c| c.dispatch_positions.get(0).cloned())
                    .unwrap_or(cur);
                let dy2 = dispatch.1 - cur.1;
                emit_move_axis(&mut out, &mut t, cid, dy2, "y");
                let dx2 = dispatch.0 - cur.0;
                emit_move_axis(&mut out, &mut t, cid, dx2, "x");
                cur = dispatch;

                // unload at crane
                out.push(format!("{} {} unload", t, cid)); t += 1;
            }
        }
    }

    out
}
