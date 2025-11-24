use crate::model::*;
use anyhow::{anyhow, Result};
use std::collections::HashMap;

/* Simple deterministic planner that enforces:
 - Dispatch ops happen with carrier horizontal (Right)
 - Storage ops happen with carrier vertical (Down)
 - Carrier must not rotate to horizontal while intersecting any Storage
 - When rotation to horizontal would be needed while intersecting storage,
   the planner first moves the carrier up out of the storage area.
 - No highway heuristic: go directly to coordinates requested.
*/

struct WorldState {
    container_locs: HashMap<Id, Loc>,
    storage_stacks: Vec<Vec<Id>>,
    car: CarState,
    out: Vec<String>,
}

#[derive(Debug, Clone)]
struct CarState {
    id: Id,
    pos: Point,
    dir: Direction,
    carrying: Option<Id>,
    t: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Loc {
    InStorage(usize),
    OnShip,
    OnCarrier,
}

fn dir_str(d: Direction) -> &'static str {
    match d { Direction::Up => "up", Direction::Down => "down", Direction::Left => "left", Direction::Right => "right" }
}

fn carrier_rect_at(bl: Point, dir: Direction) -> Rect {
    match dir {
        Direction::Up | Direction::Down => Rect { x1: bl.x, y1: bl.y, x2: bl.x + 3, y2: bl.y + 7 },
        Direction::Left | Direction::Right => Rect { x1: bl.x, y1: bl.y, x2: bl.x + 7, y2: bl.y + 3 },
    }
}

fn intersect(a: Rect, b: Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}

fn do_face(world: &mut WorldState, new_dir: Direction) {
    if world.car.dir == new_dir { return; }
    world.out.push(format!("{} face {}", world.car.t, dir_str(new_dir)));
    world.car.t += 1;
    let old = world.car.dir;
    let (dx, dy) = match (old, new_dir) {
        (Direction::Down, Direction::Right) => (2, -2), (Direction::Right, Direction::Up) => (2, 2),
        (Direction::Up, Direction::Left) => (-2, 2), (Direction::Left, Direction::Down) => (-2, -2),
        (Direction::Down, Direction::Left) => (-2, -2), (Direction::Left, Direction::Up) => (-2, 2),
        (Direction::Up, Direction::Right) => (2, 2), (Direction::Right, Direction::Down) => (2, -2),
        _ => (0, 0),
    };
    world.car.dir = new_dir;
    world.car.pos.x += dx;
    world.car.pos.y += dy;
}

fn face_dir(world: &mut WorldState, new_dir: Direction, inst: &Instance) {
    if world.car.dir == new_dir { return; }

    // If rotating to horizontal while currently intersecting any storage, move up first.
    if new_dir == Direction::Left || new_dir == Direction::Right {
        let rect = carrier_rect_at(world.car.pos, world.car.dir);
        let mut max_exit_y: Option<i32> = None;
        for s in &inst.storages {
            if intersect(rect, s.rect) {
                let exit_y = s.rect.y2 + 1; // BL.y just above storage
                max_exit_y = Some(max_exit_y.map_or(exit_y, |v| v.max(exit_y)));
            }
        }
        if let Some(y) = max_exit_y {
            // move vertically up until clear
            // face Up then move
            do_face(world, Direction::Up);
            execute_move_y(world, y, inst);
        }
    }

    do_face(world, new_dir);
}

fn execute_move_y(world: &mut WorldState, target_y: i32, inst: &Instance) {
    let dy = target_y - world.car.pos.y;
    if dy == 0 { return; }
    let needed = if dy > 0 { Direction::Up } else { Direction::Down };
    if world.car.dir != needed {
        face_dir(world, needed, inst);
    }
    let steps = dy.abs();
    world.out.push(format!("{} move {}", world.car.t, steps));
    world.car.t += steps;
    world.car.pos.y = target_y;
}

fn execute_move_x(world: &mut WorldState, target_x: i32, inst: &Instance) {
    let dx = target_x - world.car.pos.x;
    if dx == 0 { return; }
    let needed = if dx > 0 { Direction::Right } else { Direction::Left };
    if world.car.dir != needed {
        face_dir(world, needed, inst);
    }
    let steps = dx.abs();
    world.out.push(format!("{} move {}", world.car.t, steps));
    world.car.t += steps;
    world.car.pos.x = target_x;
}

fn goto_dispatch(world: &mut WorldState, dispatch: &Dispatch, inst: &Instance) -> Result<()> {
    let tx = dispatch.rect.x1;
    let ty = dispatch.rect.y1;
    execute_move_x(world, tx, inst);
    execute_move_y(world, ty, inst);
    face_dir(world, Direction::Right, inst);
    Ok(())
}

fn goto_storage(world: &mut WorldState, storage: &Storage, inst: &Instance) -> Result<()> {
    let tx = storage.rect.x1 - 1;
    let ty = storage.rect.y1 - 2;
    execute_move_x(world, tx, inst);
    if world.car.dir != Direction::Up && world.car.dir != Direction::Down {
        face_dir(world, Direction::Up, inst);
    }
    execute_move_y(world, ty, inst);
    face_dir(world, Direction::Down, inst);
    Ok(())
}

fn action_load(world: &mut WorldState, cid: Id) -> Result<()> {
    if world.car.carrying.is_some() { return Err(anyhow!("Load falhou: Carrier cheio")); }
    world.out.push(format!("{} load", world.car.t));
    world.car.t += 1;
    world.car.carrying = Some(cid);
    world.container_locs.insert(cid, Loc::OnCarrier);
    Ok(())
}

fn action_unload(world: &mut WorldState) -> Result<Id> {
    let cid = world.car.carrying.ok_or_else(|| anyhow!("Unload falhou: Carrier vazio"))?;
    world.out.push(format!("{} unload", world.car.t));
    world.car.t += 1;
    world.car.carrying = None;
    Ok(cid)
}

pub fn plan_sequential(inst: &Instance) -> Result<Vec<String>> {
    let mut locs = HashMap::new();
    for (idx, stack) in inst.storage_stacks.iter().enumerate() {
        for &cid in stack { locs.insert(cid, Loc::InStorage(idx)); }
    }
    if inst.carriers.is_empty() { return Ok(Vec::new()); }
    let car0 = &inst.carriers[0];

    let init_rect = carrier_rect_at(car0.bl, car0.dir);
    for s in &inst.storages {
        if intersect(init_rect, s.rect) && (car0.dir == Direction::Left || car0.dir == Direction::Right) {
            return Err(anyhow!("Carrier começa horizontal dentro da storage (inválido)"));
        }
    }

    let mut world = WorldState {
        container_locs: locs,
        storage_stacks: inst.storage_stacks.clone(),
        car: CarState { id: car0.id, pos: car0.bl, dir: car0.dir, carrying: car0.carrying, t: 0 },
        out: Vec::new(),
    };
    world.out.push(format!("carrier {}", world.car.id));

    for demand in &inst.demands {
        match *demand {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                let d = inst.dispatches.iter().find(|x| x.id == dispatch_id)
                    .ok_or_else(|| anyhow!("Dispatch {} inexistente", dispatch_id))?;
                goto_dispatch(&mut world, d, inst)?;
                action_load(&mut world, container_id)?;

                let s_idx = inst.storages.iter().position(|x| x.id == storage_id)
                    .ok_or_else(|| anyhow!("Storage {} inexistente", storage_id))?;
                if world.storage_stacks[s_idx].len() >= 2 {
                    return Err(anyhow!("Falha de Planeamento: Storage {} já tem 2 contentores", storage_id));
                }
                let s = &inst.storages[s_idx];
                goto_storage(&mut world, s, inst)?;
                action_unload(&mut world)?;
                world.storage_stacks[s_idx].push(container_id);
                world.container_locs.insert(container_id, Loc::InStorage(s_idx));
            }
            Demand::Load { dispatch_id, container_id } => {
                let loc = *world.container_locs.get(&container_id)
                    .ok_or_else(|| anyhow!("Contentor {} perdido", container_id))?;
                match loc {
                    Loc::InStorage(s_idx) => {
                        let stack = &world.storage_stacks[s_idx];
                        if stack.len() > 1 && stack[0] == container_id {
                            let top_cid = stack[1];
                            let s = &inst.storages[s_idx];
                            goto_storage(&mut world, s, inst)?;
                            action_load(&mut world, top_cid)?;
                            world.storage_stacks[s_idx].pop();

                            let free_idx = world.storage_stacks.iter().position(|st| st.len() < 2)
                                .ok_or_else(|| anyhow!("Sem espaço livre para reshuffle"))?;
                            let free_s = &inst.storages[free_idx];
                            goto_storage(&mut world, free_s, inst)?;
                            action_unload(&mut world)?;
                            world.storage_stacks[free_idx].push(top_cid);
                            world.container_locs.insert(top_cid, Loc::InStorage(free_idx));
                        }

                        let s = &inst.storages[s_idx];
                        goto_storage(&mut world, s, inst)?;
                        action_load(&mut world, container_id)?;
                        world.storage_stacks[s_idx].retain(|&x| x != container_id);

                        let d = inst.dispatches.iter().find(|x| x.id == dispatch_id)
                            .ok_or_else(|| anyhow!("Dispatch {} inexistente", dispatch_id))?;
                        goto_dispatch(&mut world, d, inst)?;
                        action_unload(&mut world)?;
                        world.container_locs.insert(container_id, Loc::OnShip);
                    }
                    Loc::OnShip => { }
                    Loc::OnCarrier => return Err(anyhow!("Contentor já no carrier??")),
                }
            }
        }
    }
    Ok(world.out)
}