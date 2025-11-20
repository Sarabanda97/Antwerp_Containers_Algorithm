use crate::model::*;
use anyhow::{anyhow, Result};
use std::collections::{HashMap, VecDeque};

/* =========================== Helpers básicos =========================== */

#[derive(Debug, Clone, Copy)]
enum Side { Up, Down, Left, Right }

fn map_rect(inst: &Instance) -> Rect {
    Rect { x1: 0, y1: 0, x2: inst.width, y2: inst.height }
}


fn dispatch_rect(inst: &Instance, did: Id) -> Rect {
    inst.dispatches
        .iter()
        .find(|d| d.id == did)
        .expect("dispatch_id inexistente")
        .rect
}

fn storage_rect(inst: &Instance, sid: Id) -> Rect {
    inst.storages
        .iter()
        .find(|s| s.id == sid)
        .expect("storage_id inexistente")
        .rect
}

/// staging imediatamente fora do rect, no lado indicado, com margem 1
fn staging_point_outside(rect: Rect, side: Side, inst: &Instance) -> Point {
    let map = map_rect(inst);
    let margin = 1;

    let p = match side {
        Side::Up => Point { x: rect.x1, y: rect.y2 + margin },
        Side::Down => Point { x: rect.x1, y: rect.y1 - margin },
        Side::Left => Point { x: rect.x1 - margin, y: rect.y1 },
        Side::Right => Point { x: rect.x2 + margin, y: rect.y1 },
    };

    Point {
        x: p.x.clamp(map.x1, map.x2),
        y: p.y.clamp(map.y1, map.y2),
    }
}

/// escolhe staging mais perto do ponto atual (Manhattan)
fn best_staging(rect: Rect, inst: &Instance, from: Point) -> Point {
    use Side::*;
    let candidates = [
        staging_point_outside(rect, Up, inst),
        staging_point_outside(rect, Down, inst),
        staging_point_outside(rect, Left, inst),
        staging_point_outside(rect, Right, inst),
    ];
    *candidates
        .iter()
        .min_by_key(|p| (p.x - from.x).abs() + (p.y - from.y).abs())
        .unwrap()
}

/* =========================== Geometria do carrier =========================== */

fn dim_for(dir: Direction) -> (i32, i32) {
    match dir {
        Direction::Up | Direction::Down => (4, 8),
        Direction::Left | Direction::Right => (8, 4),
    }
}

fn carrier_rect_at(bl: Point, dir: Direction) -> Rect {
    let (w, h) = dim_for(dir);
    Rect { x1: bl.x, y1: bl.y, x2: bl.x + w - 1, y2: bl.y + h - 1 }
}

fn inside_map(rect: Rect, inst: &Instance) -> bool {
    let m = map_rect(inst);
    rect.x1 >= m.x1 && rect.y1 >= m.y1 && rect.x2 <= m.x2 && rect.y2 <= m.y2
}

fn dir_str(d: Direction) -> &'static str {
    match d {
        Direction::Up => "up",
        Direction::Right => "right",
        Direction::Down => "down",
        Direction::Left => "left",
    }
}

fn dir_left(d: Direction) -> Direction {
    use Direction::*;
    match d {
        Up => Left,
        Left => Down,
        Down => Right,
        Right => Up,
    }
}

fn dir_right(d: Direction) -> Direction {
    use Direction::*;
    match d {
        Up => Right,
        Right => Down,
        Down => Left,
        Left => Up,
    }
}

/// deslocamento do BL quando rodamos 90° com centro fixo (carrier 4×8 / 8×4)
fn bl_delta_on_face(from: Direction, to: Direction) -> (i32, i32) {
    use Direction::*;
    match (from, to) {
        // clockwise
        (Up, Right)    => ( 2,  2),
        (Right, Down)  => ( 2, -2),
        (Down, Left)   => (-2, -2),
        (Left, Up)     => (-2,  2),
        // counter-clockwise
        (Right, Up)    => (-2, -2),
        (Down, Right)  => (-2,  2),
        (Left, Down)   => ( 2,  2),
        (Up, Left)     => ( 2, -2),
        // mesma direção
        _ => (0, 0),
    }
}

fn step_delta(dir: Direction) -> (i32, i32) {
    use Direction::*;
    match dir {
        Right => (1, 0),
        Left  => (-1, 0),
        Up    => (0, 1),
        Down  => (0, -1),
    }
}

fn intersect(a: Rect, b: Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}



fn is_rect_free(rect: Rect, inst: &Instance) -> bool {
    // proibimos apenas interseção com storages e dispatches.
    // a área da grua (crane.rect) pode ser cruzada, desde que
    // não entremos na dispatch section (4x2).
    !inst.storages.iter().any(|s| intersect(rect, s.rect))
        && !inst.dispatches.iter().any(|d| intersect(rect, d.rect))
}


/* =========================== Estado do carrier =========================== */

#[derive(Clone, Copy, Debug)]
struct CarState {
    id: Id,
    pos: Point,          // bottom-left
    dir: Direction,      // direção atual
    t: i32,              // tempo atual
    carrying: Option<Id> // contentor a bordo
}

impl CarState {
    fn new_from(c: &Carrier) -> Self {
        Self {
            id: c.id,
            pos: c.bl,
            dir: c.dir,
            t: 0,
            carrying: c.carrying,
        }
    }
}

/* =========================== BFS de movimento =========================== */

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct State {
    pos: Point,
    dir: Direction,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MoveOp {
    TurnLeft,
    TurnRight,
    Step, // andar 1 célula em frente
}

fn bfs_path(inst: &Instance, start: State, goal_pos: Point) -> Result<Vec<MoveOp>> {
    use std::collections::HashMap;
    let mut q = VecDeque::new();
    let mut parent: HashMap<State, (State, MoveOp)> = HashMap::new();

    q.push_back(start);

    let mut goal_state: Option<State> = None;

    while let Some(s) = q.pop_front() {
        if s.pos.x == goal_pos.x && s.pos.y == goal_pos.y {
            goal_state = Some(s);
            break;
        }

        // 1) Turn left
        {
            let ndir = dir_left(s.dir);
            let (dx, dy) = bl_delta_on_face(s.dir, ndir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, ndir);
            if inside_map(rect, inst) && is_rect_free(rect, inst) {
                let ns = State { pos: npos, dir: ndir };
                if !parent.contains_key(&ns) && ns != start {
                    parent.insert(ns, (s, MoveOp::TurnLeft));
                    q.push_back(ns);
                }
            }
        }

        // 2) Turn right
        {
            let ndir = dir_right(s.dir);
            let (dx, dy) = bl_delta_on_face(s.dir, ndir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, ndir);
            if inside_map(rect, inst) && is_rect_free(rect, inst) {
                let ns = State { pos: npos, dir: ndir };
                if !parent.contains_key(&ns) && ns != start {
                    parent.insert(ns, (s, MoveOp::TurnRight));
                    q.push_back(ns);
                }
            }
        }

        // 3) Step forward
        {
            let (dx, dy) = step_delta(s.dir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, s.dir);
            if inside_map(rect, inst) && is_rect_free(rect, inst) {
                let ns = State { pos: npos, dir: s.dir };
                if !parent.contains_key(&ns) && ns != start {
                    parent.insert(ns, (s, MoveOp::Step));
                    q.push_back(ns);
                }
            }
        }
    }

    let goal = goal_state.ok_or_else(|| anyhow!("BFS não encontrou caminho até {:?}", goal_pos))?;

    // reconstruir caminho de MoveOp
    let mut ops_rev: Vec<MoveOp> = Vec::new();
    let mut cur = goal;
    while cur != start {
        if let Some((prev, op)) = parent.get(&cur) {
            ops_rev.push(*op);
            cur = *prev;
        } else {
            return Err(anyhow!("Erro interno a reconstruir BFS"));
        }
    }
    ops_rev.reverse();
    Ok(ops_rev)
}

/// aplica uma sequência de MoveOp ao CarState, emitindo linhas no formato `<t> <ação> [arg]`
fn apply_ops(out: &mut Vec<String>, car: &mut CarState, ops: &[MoveOp]) {
    let mut i = 0;
    while i < ops.len() {
        match ops[i] {
            MoveOp::TurnLeft => {
                let new_dir = dir_left(car.dir);
                out.push(format!("{} face {}", car.t, dir_str(new_dir)));
                car.t += 1;
                let (dx, dy) = bl_delta_on_face(car.dir, new_dir);
                car.pos.x += dx;
                car.pos.y += dy;
                car.dir = new_dir;
                i += 1;
            }
            MoveOp::TurnRight => {
                let new_dir = dir_right(car.dir);
                out.push(format!("{} face {}", car.t, dir_str(new_dir)));
                car.t += 1;
                let (dx, dy) = bl_delta_on_face(car.dir, new_dir);
                car.pos.x += dx;
                car.pos.y += dy;
                car.dir = new_dir;
                i += 1;
            }
            MoveOp::Step => {
                // agrupar passos consecutivos numa única "move k"
                let mut count = 1;
                while i + count < ops.len() && matches!(ops[i + count], MoveOp::Step) {
                    count += 1;
                }
                out.push(format!("{} move {}", car.t, count));
                car.t += count as i32;

                let (dx, dy) = step_delta(car.dir);
                car.pos.x += dx * (count as i32);
                car.pos.y += dy * (count as i32);

                i += count;
            }
        }
    }
}

/// caminho até target usando BFS, depois aplica as ações
fn go_to(out: &mut Vec<String>, car: &mut CarState, target: Point, inst: &Instance) -> Result<()> {
    let start = State { pos: car.pos, dir: car.dir };
    let ops = bfs_path(inst, start, target)?;
    apply_ops(out, car, &ops);
    Ok(())
}

/* ===================== Operações load/unload (restrições) ===================== */

fn perform_load(out: &mut Vec<String>, car: &mut CarState, container_id: Id) -> Result<()> {
    if car.carrying.is_some() {
        return Err(anyhow!("Carrier {} já está a transportar um contentor", car.id));
    }
    out.push(format!("{} load", car.t));
    car.t += 1;
    car.carrying = Some(container_id);
    Ok(())
}

fn perform_unload(out: &mut Vec<String>, car: &mut CarState) -> Result<Id> {
    let cid = car.carrying.ok_or_else(|| anyhow!("Carrier {} está vazio", car.id))?;
    out.push(format!("{} unload", car.t));
    car.t += 1;
    car.carrying = None;
    Ok(cid)
}

/* ===================== Inventário de contentores ===================== */

#[derive(Clone, Copy)]
enum Loc {
    InStorage(Id),
    OnShip,
    OnCarrier(Id),
}

fn build_initial_locations(inst: &Instance) -> HashMap<Id, Loc> {
    let mut loc = HashMap::new();
    for (idx, stack) in inst.storage_stacks.iter().enumerate() {
        if let Some(storage) = inst.storages.get(idx) {
            let sid = storage.id;
            for &cid in stack {
                loc.insert(cid, Loc::InStorage(sid));
            }
        }
    }
    loc
}

/* =========================== Planner sequencial (1 carrier, genérico) =========================== */

pub fn plan_sequential(inst: &Instance) -> Result<Vec<String>> {
    let mut out: Vec<String> = Vec::new();

    if inst.carriers.is_empty() {
        return Ok(out);
    }

    // MVP: só o primeiro carrier
    let base_car = &inst.carriers[0];
    let mut car = CarState::new_from(base_car);

    // header no formato pedido
    out.push(format!("carrier {}", car.id));

    // estado local: stacks (capacidade 2) e localização dos contentores
    let mut stacks = inst.storage_stacks.clone(); // idx -> Vec<container_id>
    let storage_idx_by_id: HashMap<Id, usize> =
        inst.storages.iter().enumerate().map(|(i, s)| (s.id, i)).collect();
    let mut where_is: HashMap<Id, Loc> = build_initial_locations(inst);

    for d in &inst.demands {
        match *d {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // 1) staging do dispatch (lado mais perto)
                let drect = dispatch_rect(inst, dispatch_id);
                let staging_d = best_staging(drect, inst, car.pos);
                go_to(&mut out, &mut car, staging_d, inst)?;

                // load do navio
                perform_load(&mut out, &mut car, container_id)?;

                // 2) staging do storage
                let srect = storage_rect(inst, storage_id);
                let staging_s = best_staging(srect, inst, car.pos);
                go_to(&mut out, &mut car, staging_s, inst)?;

                // capacidade 2 + atualizar stack
                let sidx = *storage_idx_by_id
                    .get(&storage_id)
                    .ok_or_else(|| anyhow!("storage {} não encontrado", storage_id))?;
                if stacks[sidx].len() >= 2 {
                    return Err(anyhow!("Storage {} cheia (capacidade 2)", storage_id));
                }
                let dropped = perform_unload(&mut out, &mut car)?;
                stacks[sidx].push(dropped);
                where_is.insert(dropped, Loc::InStorage(storage_id));
            }

            Demand::Load { dispatch_id, container_id } => {
                // 1) descobrir storage actual do contentor
                let sid = match where_is.get(&container_id).copied() {
                    Some(Loc::InStorage(sid)) => sid,
                    Some(Loc::OnCarrier(cid)) =>
                        return Err(anyhow!("Container {} já está no carrier {}", container_id, cid)),
                    Some(Loc::OnShip) =>
                        return Err(anyhow!("Container {} já está no navio", container_id)),
                    None =>
                        return Err(anyhow!("Localização desconhecida do contentor {}", container_id)),
                };

                // 2) garantir que está no topo da stack (MVP não reempilha)
                let sidx = *storage_idx_by_id
                    .get(&sid)
                    .ok_or_else(|| anyhow!("storage {} não encontrado", sid))?;
                if stacks[sidx].last().copied() != Some(container_id) {
                    return Err(anyhow!(
                        "Container {} não está no topo da storage {} (MVP não reempilha)",
                        container_id, sid
                    ));
                }

                // staging do storage
                let srect = storage_rect(inst, sid);
                let staging_s = best_staging(srect, inst, car.pos);
                go_to(&mut out, &mut car, staging_s, inst)?;

                // load do storage
                perform_load(&mut out, &mut car, container_id)?;
                stacks[sidx].pop();
                where_is.insert(container_id, Loc::OnCarrier(car.id));

                // 3) staging do dispatch e unload para o navio
                let drect = dispatch_rect(inst, dispatch_id);
                let staging_d = best_staging(drect, inst, car.pos);
                go_to(&mut out, &mut car, staging_d, inst)?;

                let dropped = perform_unload(&mut out, &mut car)?;
                debug_assert_eq!(dropped, container_id);
                where_is.insert(dropped, Loc::OnShip);
            }
        }
    }

    Ok(out)
}
