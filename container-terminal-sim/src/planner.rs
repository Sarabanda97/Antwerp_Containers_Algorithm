use crate::model::*;
use anyhow::{anyhow, Result};
use std::collections::{HashMap, VecDeque, HashSet, BinaryHeap};

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

fn bfs_path(inst: &Instance, start: State, goal_pos: Point, allow_rotations: bool) -> Result<Vec<MoveOp>> {
    use std::collections::HashMap;
    use std::cmp::Ordering;

    #[derive(Eq, PartialEq)]
    struct Node {
        f: i32,
        g: i32,
        state: State,
    }

    impl Ord for Node {
        fn cmp(&self, other: &Self) -> Ordering {
            // reverse order: we want smallest f,g to be popped first from BinaryHeap
            other.f.cmp(&self.f).then_with(|| other.g.cmp(&self.g))
        }
    }
    impl PartialOrd for Node {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> { Some(self.cmp(other)) }
    }

    let mut parent: HashMap<State, (State, MoveOp)> = HashMap::new();
    let mut g_score: HashMap<State, i32> = HashMap::new();
    // logs para diagnóstico
    let mut logs: Vec<String> = Vec::new();
    let mut expansions: usize = 0;
    let max_logs: usize = 200;

    // trivial case: already at goal position
    if start.pos.x == goal_pos.x && start.pos.y == goal_pos.y {
        return Ok(Vec::new());
    }

    // debug: informar início do A*
    eprintln!("A* START from pos=({},{}) dir={:?} to goal=({},{})", start.pos.x, start.pos.y, start.dir, goal_pos.x, goal_pos.y);

    let mut open: BinaryHeap<Node> = BinaryHeap::new();
    let h0 = (start.pos.x - goal_pos.x).abs() + (start.pos.y - goal_pos.y).abs();
    g_score.insert(start, 0);
    open.push(Node { f: h0, g: 0, state: start });

    while let Some(node) = open.pop() {
        let s = node.state;
        let g = node.g;

        // outdated entry
        if let Some(&best_g) = g_score.get(&s) {
            if g != best_g { continue; }
        }

        if expansions < max_logs {
            logs.push(format!("EXPAND {} state pos=({},{}) dir={:?}", expansions, s.pos.x, s.pos.y, s.dir));
        }
        expansions += 1;

        if s.pos.x == goal_pos.x && s.pos.y == goal_pos.y {
            // imprimir logs de diagnóstico quando encontrar goal
            if !logs.is_empty() {
                eprintln!("--- A* DIAGNOSTIC LOG (first {} entries) ---", logs.len());
                for l in &logs { eprintln!("{}", l); }
            }

            // reconstruir caminho
            let mut ops_rev: Vec<MoveOp> = Vec::new();
            let mut cur = s;
            while cur != start {
                if let Some((prev, op)) = parent.get(&cur) {
                    ops_rev.push(*op);
                    cur = *prev;
                } else {
                    return Err(anyhow!("Erro interno a reconstruir A*"));
                }
            }
            ops_rev.reverse();
            return Ok(ops_rev);
        }

        // helper to consider a candidate neighbor
        let consider = |ns: State, op: MoveOp, rect: Rect, s: State, g: i32, inst: &Instance, logs: &mut Vec<String>, expansions: usize, max_logs: usize, parent: &mut HashMap<State, (State, MoveOp)>, g_score: &mut HashMap<State, i32>, open: &mut BinaryHeap<Node>, start: State, goal_pos: Point| {
            // inside map?
            if !inside_map(rect, inst) {
                if expansions < max_logs {
                    logs.push(format!("REJECT {:?} out_of_map candidate_pos=({},{}) dir={:?} rect=({},{},{},{})", op, ns.pos.x, ns.pos.y, ns.dir, rect.x1, rect.y1, rect.x2, rect.y2));
                }
                return;
            }

            if !is_rect_free(rect, inst) {
                let cur_rect = carrier_rect_at(s.pos, s.dir);
                let cur_overlaps = !is_rect_free(cur_rect, inst);
                if !(cur_overlaps || (s.pos == start.pos && s.dir == start.dir)) {
                    if expansions < max_logs {
                        let mut why = String::from("collision");
                        for sarea in &inst.storages {
                            if intersect(rect, sarea.rect) { why = format!("storage {}", sarea.id); break; }
                        }
                        for darea in &inst.dispatches {
                            if intersect(rect, darea.rect) { why = format!("dispatch {}", darea.id); break; }
                        }
                        logs.push(format!("REJECT {:?} collision({}) candidate_pos=({},{}) dir={:?} rect=({},{},{},{})", op, why, ns.pos.x, ns.pos.y, ns.dir, rect.x1, rect.y1, rect.x2, rect.y2));
                    }
                    return;
                }
            }

            // tentative g
            let tentative_g = g + 1;
            let best = g_score.get(&ns).copied().unwrap_or(i32::MAX);
            if tentative_g < best {
                parent.insert(ns, (s, op));
                g_score.insert(ns, tentative_g);
                let h = (ns.pos.x - goal_pos.x).abs() + (ns.pos.y - goal_pos.y).abs();
                let f = tentative_g + h;
                open.push(Node { f, g: tentative_g, state: ns });
                if expansions < max_logs {
                    logs.push(format!("PUSH {:?} pos=({},{}) dir={:?} g={} f={}", op, ns.pos.x, ns.pos.y, ns.dir, tentative_g, f));
                }
            }
        };

        // 1) Turn left (if allowed)
        if allow_rotations {
            let ndir = dir_left(s.dir);
            let (dx, dy) = bl_delta_on_face(s.dir, ndir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, ndir);
            let ns = State { pos: npos, dir: ndir };
            // For rotations, ensure the swept area between current rect and new rect is also free.
            let cur_rect = carrier_rect_at(s.pos, s.dir);
            let sweep = Rect {
                x1: cur_rect.x1.min(rect.x1), y1: cur_rect.y1.min(rect.y1),
                x2: cur_rect.x2.max(rect.x2), y2: cur_rect.y2.max(rect.y2)
            };
            let mut rotation_blocked = false;
            if inst.storages.iter().any(|st| intersect(sweep, st.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect)) {
                rotation_blocked = true;
            }
            if rotation_blocked {
                if expansions < max_logs {
                    logs.push(format!("REJECT TurnLeft rotation_blocked candidate_pos=({},{}) dir={:?} sweep=({},{},{},{})", npos.x, npos.y, ndir, sweep.x1, sweep.y1, sweep.x2, sweep.y2));
                }
            } else {
                consider(ns, MoveOp::TurnLeft, rect, s, g, inst, &mut logs, expansions, max_logs, &mut parent, &mut g_score, &mut open, start, goal_pos);
            }
        }

        // 2) Turn right (if allowed)
        if allow_rotations {
            let ndir = dir_right(s.dir);
            let (dx, dy) = bl_delta_on_face(s.dir, ndir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, ndir);
            let ns = State { pos: npos, dir: ndir };
            // check rotation sweep area as for TurnLeft
            let cur_rect = carrier_rect_at(s.pos, s.dir);
            let sweep = Rect {
                x1: cur_rect.x1.min(rect.x1), y1: cur_rect.y1.min(rect.y1),
                x2: cur_rect.x2.max(rect.x2), y2: cur_rect.y2.max(rect.y2)
            };
            let mut rotation_blocked = false;
            if inst.storages.iter().any(|st| intersect(sweep, st.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect)) {
                rotation_blocked = true;
            }
            if rotation_blocked {
                if expansions < max_logs {
                    logs.push(format!("REJECT TurnRight rotation_blocked candidate_pos=({},{}) dir={:?} sweep=({},{},{},{})", npos.x, npos.y, ndir, sweep.x1, sweep.y1, sweep.x2, sweep.y2));
                }
            } else {
                consider(ns, MoveOp::TurnRight, rect, s, g, inst, &mut logs, expansions, max_logs, &mut parent, &mut g_score, &mut open, start, goal_pos);
            }
        }

        // 3) Step forward
        {
            let (dx, dy) = step_delta(s.dir);
            let npos = Point { x: s.pos.x + dx, y: s.pos.y + dy };
            let rect = carrier_rect_at(npos, s.dir);
            let ns = State { pos: npos, dir: s.dir };
            consider(ns, MoveOp::Step, rect, s, g, inst, &mut logs, expansions, max_logs, &mut parent, &mut g_score, &mut open, start, goal_pos);
        }
    }

    // falhou
    if !logs.is_empty() {
        eprintln!("--- A* DIAGNOSTIC LOG (first {} entries) ---", logs.len());
        for l in &logs { eprintln!("{}", l); }
    }
    Err(anyhow!("A* não encontrou caminho até {:?}", goal_pos))
}

/// aplica uma sequência de MoveOp ao CarState, emitindo linhas no formato `<t> <ação> [arg]`
fn apply_ops(out: &mut Vec<String>, car: &mut CarState, ops: &[MoveOp], inst: &Instance) -> Result<()> {
    let mut i = 0;
    while i < ops.len() {
        match ops[i] {
            MoveOp::TurnLeft => {
                let new_dir = dir_left(car.dir);
                // check rotation sweep before applying
                let cur_rect = carrier_rect_at(car.pos, car.dir);
                let (dx_rot, dy_rot) = bl_delta_on_face(car.dir, new_dir);
                let npos = Point { x: car.pos.x + dx_rot, y: car.pos.y + dy_rot };
                let new_rect = carrier_rect_at(npos, new_dir);
                let sweep = Rect { x1: cur_rect.x1.min(new_rect.x1), y1: cur_rect.y1.min(new_rect.y1), x2: cur_rect.x2.max(new_rect.x2), y2: cur_rect.y2.max(new_rect.y2) };
                let blocked = inst.storages.iter().any(|s| intersect(sweep, s.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect));
                if blocked {
                    // try to move forward a few steps to create space
                    let max_attempts = 3;
                    let mut made_space = false;
                    for _ in 0..max_attempts {
                        let (sx, sy) = step_delta(car.dir);
                        let cand = Point { x: car.pos.x + sx, y: car.pos.y + sy };
                        let crect = carrier_rect_at(cand, car.dir);
                        if !inside_map(crect, inst) { break; }
                        if !is_rect_free(crect, inst) { break; }
                        // perform one step
                        out.push(format!("{} move {}", car.t, 1));
                        car.t += 1;
                        car.pos = cand;
                        // recompute sweep
                        let cur_rect = carrier_rect_at(car.pos, car.dir);
                        let npos = Point { x: car.pos.x + dx_rot, y: car.pos.y + dy_rot };
                        let new_rect = carrier_rect_at(npos, new_dir);
                        let sweep = Rect { x1: cur_rect.x1.min(new_rect.x1), y1: cur_rect.y1.min(new_rect.y1), x2: cur_rect.x2.max(new_rect.x2), y2: cur_rect.y2.max(new_rect.y2) };
                        let blocked2 = inst.storages.iter().any(|s| intersect(sweep, s.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect));
                        if !blocked2 { made_space = true; break; }
                    }
                    if !made_space {
                        return Err(anyhow!("Rotation blocked by storage/dispatch at pos {:?} dir {:?}", car.pos, car.dir));
                    }
                }
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
                // check rotation sweep
                let cur_rect = carrier_rect_at(car.pos, car.dir);
                let (dx_rot, dy_rot) = bl_delta_on_face(car.dir, new_dir);
                let npos = Point { x: car.pos.x + dx_rot, y: car.pos.y + dy_rot };
                let new_rect = carrier_rect_at(npos, new_dir);
                let sweep = Rect { x1: cur_rect.x1.min(new_rect.x1), y1: cur_rect.y1.min(new_rect.y1), x2: cur_rect.x2.max(new_rect.x2), y2: cur_rect.y2.max(new_rect.y2) };
                let blocked = inst.storages.iter().any(|s| intersect(sweep, s.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect));
                if blocked {
                    let max_attempts = 3;
                    let mut made_space = false;
                    for _ in 0..max_attempts {
                        let (sx, sy) = step_delta(car.dir);
                        let cand = Point { x: car.pos.x + sx, y: car.pos.y + sy };
                        let crect = carrier_rect_at(cand, car.dir);
                        if !inside_map(crect, inst) { break; }
                        if !is_rect_free(crect, inst) { break; }
                        out.push(format!("{} move {}", car.t, 1));
                        car.t += 1;
                        car.pos = cand;
                        let cur_rect = carrier_rect_at(car.pos, car.dir);
                        let npos = Point { x: car.pos.x + dx_rot, y: car.pos.y + dy_rot };
                        let new_rect = carrier_rect_at(npos, new_dir);
                        let sweep = Rect { x1: cur_rect.x1.min(new_rect.x1), y1: cur_rect.y1.min(new_rect.y1), x2: cur_rect.x2.max(new_rect.x2), y2: cur_rect.y2.max(new_rect.y2) };
                        let blocked2 = inst.storages.iter().any(|s| intersect(sweep, s.rect)) || inst.dispatches.iter().any(|d| intersect(sweep, d.rect));
                        if !blocked2 { made_space = true; break; }
                    }
                    if !made_space {
                        return Err(anyhow!("Rotation blocked by storage/dispatch at pos {:?} dir {:?}", car.pos, car.dir));
                    }
                }
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
                // ensure steps are valid
                let (dx, dy) = step_delta(car.dir);
                let cand = Point { x: car.pos.x + dx * (count as i32), y: car.pos.y + dy * (count as i32) };
                let crect = carrier_rect_at(cand, car.dir);
                if !inside_map(crect, inst) {
                    return Err(anyhow!("Step would go out of map to {:?}", cand));
                }
                if !is_rect_free(crect, inst) {
                    return Err(anyhow!("Step would collide at {:?}", cand));
                }
                out.push(format!("{} move {}", car.t, count));
                car.t += count as i32;
                car.pos.x = cand.x;
                car.pos.y = cand.y;
                i += count;
            }
        }
    }
    Ok(())
}

/// caminho até target usando BFS, depois aplica as ações
fn go_to(out: &mut Vec<String>, car: &mut CarState, target: Point, inst: &Instance) -> Result<()> {
    let start = State { pos: car.pos, dir: car.dir };
    let ops = bfs_path(inst, start, target, true)?;
    apply_ops(out, car, &ops, inst)?;
    Ok(())
}

/// go_to variant that forbids rotations (used for initial escape where we must not rotate)
fn go_to_no_rotate(out: &mut Vec<String>, car: &mut CarState, target: Point, inst: &Instance) -> Result<()> {
    let start = State { pos: car.pos, dir: car.dir };
    let ops = bfs_path(inst, start, target, false)?;
    apply_ops(out, car, &ops, inst)?;
    Ok(())
}

/// Try multiple staging candidates (sides) for a target rect, ordered by Manhattan distance.
fn go_to_staging(out: &mut Vec<String>, car: &mut CarState, rect: Rect, inst: &Instance) -> Result<()> {
    use Side::*;
    let sides = [Up, Down, Left, Right];
    let mut cand: Vec<(Point, Side)> = sides.iter().map(|&s| (staging_point_outside(rect, s, inst), s)).collect();
    cand.sort_by_key(|(p, _)| (p.x - car.pos.x).abs() + (p.y - car.pos.y).abs());

    for (p, _s) in cand {
        match go_to(out, car, p, inst) {
            Ok(()) => return Ok(()),
            Err(e) => {
                eprintln!("staging candidate {:?} failed: {}", p, e);
                // try next
            }
        }
    }
    // fallback: try to move the carrier a few cells down to escape an overlapped start,
    // then re-attempt staging candidates. This matches the requested "go down first" behaviour.
    for step in 1..=3 {
        let new_bl = Point { x: car.pos.x, y: car.pos.y - step };
        let rect_new = carrier_rect_at(new_bl, car.dir);
        if !inside_map(rect_new, inst) { break; }
        // attempt to move there (go_to will allow escape if current overlaps)
        match go_to(out, car, new_bl, inst) {
            Ok(()) => {
                // after moving down, retry staging candidates
                let mut cand2: Vec<(Point, Side)> = sides.iter().map(|&s| (staging_point_outside(rect, s, inst), s)).collect();
                cand2.sort_by_key(|(p, _)| (p.x - car.pos.x).abs() + (p.y - car.pos.y).abs());
                for (p2, _s2) in cand2 {
                    match go_to(out, car, p2, inst) {
                        Ok(()) => return Ok(()),
                        Err(e) => eprintln!("staging candidate after escape {:?} failed: {}", p2, e),
                    }
                }
            }
            Err(e) => {
                eprintln!("attempted local escape to {:?} failed: {}", new_bl, e);
            }
        }
    }
    // extended fallback: try nearby free BL positions around the rect within a small radius
    let mut candidates: Vec<Point> = Vec::new();
    let radius = 5;
    for x in (rect.x1 - radius)..=(rect.x2 + radius) {
        for y in (rect.y1 - radius)..=(rect.y2 + radius) {
            let p = Point { x, y };
            // compute min manhattan distance to the rect border
            let dx = if p.x < rect.x1 { rect.x1 - p.x } else if p.x > rect.x2 { p.x - rect.x2 } else { 0 };
            let dy = if p.y < rect.y1 { rect.y1 - p.y } else if p.y > rect.y2 { p.y - rect.y2 } else { 0 };
            let dist = dx + dy;
            if dist == 0 || dist > radius { continue; }
            let crect = carrier_rect_at(p, car.dir);
            if !inside_map(crect, inst) { continue; }
            if !is_rect_free(crect, inst) { continue; }
            candidates.push(p);
        }
    }
    candidates.sort_by_key(|p| (p.x - car.pos.x).abs() + (p.y - car.pos.y).abs());
    for p in candidates {
        match go_to(out, car, p, inst) {
            Ok(()) => return Ok(()),
            Err(e) => eprintln!("fallback candidate {:?} failed: {}", p, e),
        }
    }

    Err(anyhow!("Nenhum staging alcançável para rect {:?} a partir de State {:?}", rect, State { pos: car.pos, dir: car.dir }))
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

    // MVP: apenas o primeiro carrier
    let base_car = &inst.carriers[0];
    let mut car = CarState::new_from(base_car);

    out.push(format!("carrier {}", car.id));

    // helper: produce minimal rotations to face `d` from `car.dir`
    let mut face_ops = |car: &CarState, d: Direction| -> Vec<MoveOp> {
        let mut ops: Vec<MoveOp> = Vec::new();
        let idx = |x: Direction| match x {
            Direction::Up => 0,
            Direction::Right => 1,
            Direction::Down => 2,
            Direction::Left => 3,
        };
        let cur = idx(car.dir);
        let tgt = idx(d);
        let diff = (tgt + 4 - cur) % 4;
        match diff {
            0 => {}
            1 => ops.push(MoveOp::TurnRight),
            2 => { ops.push(MoveOp::TurnRight); ops.push(MoveOp::TurnRight); }
            3 => ops.push(MoveOp::TurnLeft),
            _ => {}
        }
        ops
    };

    // helper: naive Manhattan move (x then y)
    let mut naive_move_to = |out: &mut Vec<String>, car: &mut CarState, target: Point| -> Result<()> {
        let mut ops: Vec<MoveOp> = Vec::new();
        // horizontal move
        if target.x != car.pos.x {
            let dir_x = if target.x > car.pos.x { Direction::Right } else { Direction::Left };
            ops.extend(face_ops(car, dir_x));
            let steps = (target.x - car.pos.x).abs();
            for _ in 0..steps { ops.push(MoveOp::Step); }
            apply_ops(out, car, &ops, inst)?;
            ops.clear();
        }
        // vertical move
        if target.y != car.pos.y {
            let dir_y = if target.y > car.pos.y { Direction::Up } else { Direction::Down };
            ops.extend(face_ops(car, dir_y));
            let steps = (target.y - car.pos.y).abs();
            for _ in 0..steps { ops.push(MoveOp::Step); }
            apply_ops(out, car, &ops, inst)?;
            ops.clear();
        }
        Ok(())
    };

    // helper: try small local displacements to resolve overlap before load/unload
    let mut try_resolve_overlap = |out: &mut Vec<String>, car: &mut CarState| -> Result<()> {
        let crect = carrier_rect_at(car.pos, car.dir);
        if is_rect_free(crect, inst) { return Ok(()); }
        // first try moving down up to 10 steps (user preference)
        for step in 1..=10 {
            let cand = Point { x: car.pos.x, y: car.pos.y - step };
            let rect = carrier_rect_at(cand, car.dir);
            if !inside_map(rect, inst) { break; }
            if !is_rect_free(rect, inst) { continue; }
            if naive_move_to(out, car, cand).is_ok() {
                eprintln!("Resolved overlap by moving down to {:?}", cand);
                return Ok(());
            }
        }

        // fall back to searching nearby free BL positions (radius)
        let radius: i32 = 5;
        for r in 1..=radius {
            for dx in -r..=r {
                for dy in -r..=r {
                    if dx.abs() + dy.abs() != r { continue; }
                    let cand = Point { x: car.pos.x + dx, y: car.pos.y + dy };
                    let rect = carrier_rect_at(cand, car.dir);
                    if !inside_map(rect, inst) { continue; }
                    if !is_rect_free(rect, inst) { continue; }
                    if naive_move_to(out, car, cand).is_ok() {
                        eprintln!("Resolved overlap by moving to nearby {:?}", cand);
                        return Ok(());
                    }
                }
            }
        }

        Err(anyhow!("Unable to resolve overlap for carrier {} at pos {:?}", car.id, car.pos))
    };

    // wrapper: try naive_move_to, on failure attempt overlap resolution then retry
    let mut try_naive_to = |out: &mut Vec<String>, car: &mut CarState, target: Point| -> Result<()> {
        match naive_move_to(out, car, target) {
            Ok(()) => Ok(()),
            Err(e) => {
                eprintln!("naive_move_to failed to {:?}: {}. Trying overlap resolution and retry.", target, e);
                if try_resolve_overlap(out, car).is_ok() {
                    naive_move_to(out, car, target)
                } else {
                    Err(e)
                }
            }
        }
    };

    // naive staging: try multiple sides and small outward offsets (global behaviour)
    let mut go_to_staging_naive = |out: &mut Vec<String>, car: &mut CarState, rect: Rect| -> Result<()> {
        use Side::*;
        let sides = [Up, Down, Left, Right];
        let mut candidates: Vec<Point> = Vec::new();
        for &s in &sides {
            let base = staging_point_outside(rect, s, inst);
            // try small outward offsets (0..=3)
            for extra in 0..=3 {
                let p = match s {
                    Up => Point { x: base.x, y: base.y + extra },
                    Down => Point { x: base.x, y: base.y - extra },
                    Left => Point { x: base.x - extra, y: base.y },
                    Right => Point { x: base.x + extra, y: base.y },
                };
                let crect = carrier_rect_at(p, car.dir);
                if !inside_map(crect, inst) { continue; }
                candidates.push(p);
            }
        }
        // sort by manhattan distance
        candidates.sort_by_key(|p| (p.x - car.pos.x).abs() + (p.y - car.pos.y).abs());
        for p in candidates {
            if try_naive_to(out, car, p).is_ok() {
                return Ok(());
            }
        }
        Err(anyhow!("Nenhum staging naive alcançável para rect {:?} a partir de State {:?}", rect, State { pos: car.pos, dir: car.dir }))
    };

    // initial preference: if overlapping, try to move down a bit first (user requested)
    let start_rect = carrier_rect_at(car.pos, car.dir);
    if !is_rect_free(start_rect, inst) {
        for step in 1..=10 {
            let new_bl = Point { x: car.pos.x, y: car.pos.y - step };
            let crect = carrier_rect_at(new_bl, car.dir);
            if !inside_map(crect, inst) { break; }
            if is_rect_free(crect, inst) {
                // naive move directly (no path checks)
                let _ = naive_move_to(&mut out, &mut car, new_bl);
                break;
            }
        }
    }

    // simple inventory
    let mut stacks = inst.storage_stacks.clone();
    let storage_idx_by_id: HashMap<Id, usize> = inst.storages.iter().enumerate().map(|(i,s)| (s.id,i)).collect();
    let mut where_is: HashMap<Id, Loc> = build_initial_locations(inst);

    for d in &inst.demands {
        match *d {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // go to dispatch staging (choose nearest)
                let drect = dispatch_rect(inst, dispatch_id);
                go_to_staging_naive(&mut out, &mut car, drect)?;
                // resolve overlap if needed before load
                try_resolve_overlap(&mut out, &mut car)?;
                perform_load(&mut out, &mut car, container_id)?;

                // go to storage staging
                let srect = storage_rect(inst, storage_id);
                go_to_staging_naive(&mut out, &mut car, srect)?;
                // resolve overlap before unload to storage
                try_resolve_overlap(&mut out, &mut car)?;
                let sidx = *storage_idx_by_id.get(&storage_id).ok_or_else(|| anyhow!("storage {} não encontrado", storage_id))?;
                if stacks[sidx].len() >= 2 {
                    return Err(anyhow!("Storage {} cheia (capacidade 2)", storage_id));
                }
                let dropped = perform_unload(&mut out, &mut car)?;
                stacks[sidx].push(dropped);
                where_is.insert(dropped, Loc::InStorage(storage_id));
            }

            Demand::Load { dispatch_id, container_id } => {
                let sid = match where_is.get(&container_id).copied() {
                    Some(Loc::InStorage(sid)) => sid,
                    Some(Loc::OnCarrier(cid)) => return Err(anyhow!("Container {} já está no carrier {}", container_id, cid)),
                    Some(Loc::OnShip) => return Err(anyhow!("Container {} já está no navio", container_id)),
                    None => return Err(anyhow!("Localização desconhecida do contentor {}", container_id)),
                };
                let sidx = *storage_idx_by_id.get(&sid).ok_or_else(|| anyhow!("storage {} não encontrado", sid))?;
                if stacks[sidx].last().copied() != Some(container_id) {
                    return Err(anyhow!("Container {} não está no topo da storage {} (MVP não reempilha)", container_id, sid));
                }

                let srect = storage_rect(inst, sid);
                go_to_staging_naive(&mut out, &mut car, srect)?;
                try_resolve_overlap(&mut out, &mut car)?;
                perform_load(&mut out, &mut car, container_id)?;
                stacks[sidx].pop();
                where_is.insert(container_id, Loc::OnCarrier(car.id));

                let drect = dispatch_rect(inst, dispatch_id);
                go_to_staging_naive(&mut out, &mut car, drect)?;
                try_resolve_overlap(&mut out, &mut car)?;
                let dropped = perform_unload(&mut out, &mut car)?;
                debug_assert_eq!(dropped, container_id);
                where_is.insert(dropped, Loc::OnShip);
            }
        }
    }

    Ok(out)
}
