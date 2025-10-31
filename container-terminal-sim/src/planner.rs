use crate::model::{Instance, Demand};

fn move_axis(
    out: &mut Vec<String>,
    t: &mut i32,
    cid: i32,
    mut cur: (i32, i32),
    target: (i32, i32),
    axis: char,
) -> (i32, i32) {
    let delta = match axis {
        'x' => target.0 - cur.0,
        'y' => target.1 - cur.1,
        _ => 0,
    };
    if delta == 0 {
        return cur;
    }

    let dir = match (axis, delta > 0) {
        ('x', true) => "right",
        ('x', false) => "left",
        ('y', true) => "up",
        ('y', false) => "down",
        _ => "right",
    };

    out.push(format!("{} {} face {}", *t, cid, dir));
    *t += 1;

    let k = delta.abs();
    out.push(format!("{} {} move {}", *t, cid, k));
    *t += k;

    match axis {
        'x' => cur.0 = target.0,
        'y' => cur.1 = target.1,
        _ => {}
    }

    cur
}

fn get_crane(inst: &Instance, crane_id: i32) -> &crate::model::Crane {
    inst.cranes.iter().find(|c| c.id == crane_id).unwrap()
}

fn get_dispatch_bl(inst: &Instance, crane_id: i32) -> (i32, i32) {
    let crane = get_crane(inst, crane_id);
    crane.dispatch_positions[0]
}

fn get_storage_bl(inst: &Instance, storage_id: i32) -> (i32, i32) {
    let st = inst.storages.iter().find(|s| s.id == storage_id).unwrap();
    st.bl
}

fn find_container_storage_id(inst: &Instance, container_id: i32) -> Option<i32> {
    inst.containers_init
        .iter()
        .find(|(cid, _)| *cid == container_id)
        .map(|(_, sid)| *sid)
}

/// go to crane WITHOUT entering it: align X only, stay at current Y
fn go_to_crane_side(
    out: &mut Vec<String>,
    t: &mut i32,
    cid: i32,
    mut cur: (i32, i32),
    dispatch: (i32, i32),
) -> (i32, i32) {
    let target_x = (dispatch.0, cur.1);
    cur = move_axis(out, t, cid, cur, target_x, 'x');
    cur
}

/// approach storage from below: align X, then go to (storage_y - 1)
fn go_below_storage(
    out: &mut Vec<String>,
    t: &mut i32,
    cid: i32,
    mut cur: (i32, i32),
    storage_bl: (i32, i32),
) -> (i32, i32) {
    let (sx, sy) = storage_bl;
    // 1) align X
    cur = move_axis(out, t, cid, cur, (sx, cur.1), 'x');
    // 2) go UP but stop 1 before storage
    let safe_y = sy - 1;
    cur = move_axis(out, t, cid, cur, (sx, safe_y), 'y');
    cur
}

pub fn plan_sequential(inst: &Instance) -> Vec<String> {
    let mut out = Vec::new();
    if inst.carriers.is_empty() {
        return out;
    }

    let car = &inst.carriers[0];
    let cid = car.id;
    let mut cur = car.bl; // (8,61)
    let mut t = 0;

    // escape corridor
    out.push(format!("{} {} move 18", t, cid));
    t += 18;
    cur.1 -= 18; // now y=43

    for d in &inst.demands {
        match d {
            // ship -> storage
            Demand::Unload { crane_id, storage_id, .. } => {
                let crane = get_crane(inst, *crane_id);
                let disp = crane.dispatch_positions[0];

                // 1) park at crane side
                cur = go_to_crane_side(&mut out, &mut t, cid, cur, disp);

                // 2) load
                out.push(format!("{} {} load", t, cid));
                t += 1;

                // 3) go below target storage (never drive into racks)
                let st_bl = get_storage_bl(inst, *storage_id);
                cur = go_below_storage(&mut out, &mut t, cid, cur, st_bl);

                // 4) unload (we're just below, open field)
                out.push(format!("{} {} unload", t, cid));
                t += 1;
            }

            // storage -> ship
            Demand::Load { crane_id, container_id } => {
                // 1) go below the storage where container is
                let storage_id = find_container_storage_id(inst, *container_id)
                    .unwrap_or_else(|| inst.storages[0].id);
                let st_bl = get_storage_bl(inst, storage_id);
                cur = go_below_storage(&mut out, &mut t, cid, cur, st_bl);

                // 2) load
                out.push(format!("{} {} load", t, cid));
                t += 1;

                // 3) deliver to crane side
                let crane = get_crane(inst, *crane_id);
                let disp = crane.dispatch_positions[0];
                cur = go_to_crane_side(&mut out, &mut t, cid, cur, disp);

                // 4) unload to ship (open)
                out.push(format!("{} {} unload", t, cid));
                t += 1;
            }
        }
    }

    out
}
