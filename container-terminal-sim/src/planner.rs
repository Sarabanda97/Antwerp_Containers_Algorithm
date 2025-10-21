use crate::model::{Instance, Demand};

fn face_op(t: &mut i32, cid: i32, dir: &str, out: &mut Vec<String>) {
    out.push(format!("{} {} face {}", *t, cid, dir)); *t += 1;
}
fn move_op(t: &mut i32, cid: i32, k: i32, out: &mut Vec<String>) {
    out.push(format!("{} {} move {}", *t, cid, k)); *t += k.abs();
}
fn load_op(t: &mut i32, cid: i32, out: &mut Vec<String>) {
    out.push(format!("{} {} load", *t, cid)); *t += 1;
}
fn unload_op(t: &mut i32, cid: i32, out: &mut Vec<String>) {
    out.push(format!("{} {} unload", *t, cid)); *t += 1;
}

pub fn plan_sequential(inst: &Instance) -> Vec<String> {
    let cid = inst.carriers[0].id;
    let mut t = 0;
    let mut out = Vec::new();

    for d in &inst.demands {
        // go to crane dispatch (placeholder): face + move short
        face_op(&mut t, cid, "right", &mut out);
        move_op(&mut t, cid, 2, &mut out);

        match d {
            Demand::Unload{..} => {
                load_op(&mut t, cid, &mut out);        // pick at crane
                move_op(&mut t, cid, 3, &mut out);     // go to storage (fake)
                unload_op(&mut t, cid, &mut out);      // drop
            }
            Demand::Load{..} => {
                move_op(&mut t, cid, 3, &mut out);     // go to storage
                load_op(&mut t, cid, &mut out);        // pick
                move_op(&mut t, cid, 3, &mut out);     // go to crane
                unload_op(&mut t, cid, &mut out);      // drop at crane
            }
        }
    }
    out
}
