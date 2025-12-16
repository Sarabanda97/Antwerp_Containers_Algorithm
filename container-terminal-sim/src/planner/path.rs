use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;

use crate::model::{Direction, Id, Instance, Point, Rect};
use crate::state::CarrierState;

// -------------------- Reservation Table (dynamic obstacles) --------------------

pub struct ReservationTable {
    /// Explicit occupancy per time step.
    occupied: HashMap<i32, Vec<(Id, Rect)>>,
    /// Last known pose per carrier (persists forward in time unless overwritten).
    last: HashMap<Id, (i32, Rect)>,
}

impl ReservationTable {
    pub fn new() -> Self {
        Self {
            occupied: HashMap::new(),
            last: HashMap::new(),
        }
    }

    /// Check if rectangle `r` is collision-free at time `t` for carrier `me`.
    ///
    /// - Considers explicit reservations at time `t`.
    /// - Also treats every other carrier as occupying its *last reserved rectangle*
    ///   for all future times (important for "idle" carriers, otherwise they
    ///   disappear from the space-time world and can be collided with).
    pub fn is_free(&self, t: i32, me: Id, r: &Rect) -> bool {
        // 1) Explicit reservations at time t.
        let mut explicit_ids: Vec<Id> = Vec::new();
        if let Some(obstacles) = self.occupied.get(&t) {
            for (cid, obs) in obstacles {
                explicit_ids.push(*cid);
                if *cid != me && rect_intersects(r, obs) {
                    return false;
                }
            }
        }

        // 2) Persistent ("tail") occupancy: if a carrier has no explicit
        // reservation at time t, assume it stays at its last rectangle.
        for (cid, (lt, last_r)) in &self.last {
            if *cid == me {
                continue;
            }
            if *lt < t && !explicit_ids.contains(cid) {
                if rect_intersects(r, last_r) {
                    return false;
                }
            }
        }

        true
    }

    /// Reserve a rectangle for a specific carrier at time `t`.
    ///
    /// If multiple reservations happen for the same carrier at the same `t`,
    /// we keep them all (conservative), and `last` is updated to the last one.
    pub fn reserve(&mut self, t: i32, carrier_id: Id, r: Rect) {
        self.occupied.entry(t).or_default().push((carrier_id, r));
        self.last.insert(carrier_id, (t, r));
    }
}

// -------------------- Commands --------------------

#[derive(Clone, Debug)]
pub enum Command {
    Move   { t: i32, k: i32 },
    Face   { t: i32, dir: Direction },
    Load   { t: i32 },
    Unload { t: i32 },
}

// -------------------- Geometry helpers --------------------

const SHORT: i32 = 4;
const LONG:  i32 = 8;

fn dims(dir: Direction) -> (i32, i32) {
    match dir {
        Direction::Up | Direction::Down => (SHORT, LONG),        // 4×8 vertical
        Direction::Left | Direction::Right => (LONG, SHORT),     // 8×4 horizontal
    }
}

/// Center in "half-cell units" (x2,y2) so rotations with even sizes are exact without floats.
/// 2*center = 2*bl + (w-1, h-1)
fn center2_from_bl(bl: Point, dir: Direction) -> (i32, i32) {
    let (w, h) = dims(dir);
    (2 * bl.x + (w - 1), 2 * bl.y + (h - 1))
}

fn bl_from_center2(center2: (i32, i32), dir: Direction) -> Point {
    let (w, h) = dims(dir);
    Point {
        x: (center2.0 - (w - 1)) / 2,
        y: (center2.1 - (h - 1)) / 2,
    }
}

pub fn carrier_rect(bl: Point, dir: Direction) -> Rect {
    let (w, h) = dims(dir);
    Rect { x1: bl.x, y1: bl.y, x2: bl.x + w - 1, y2: bl.y + h - 1 }
}

fn rect_intersects(a: &Rect, b: &Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}

fn rect_within(r: &Rect, limit: &Rect) -> bool {
    r.x1 >= limit.x1 && r.y1 >= limit.y1 && r.x2 <= limit.x2 && r.y2 <= limit.y2
}

fn intersects_any_storage(inst: &Instance, r: &Rect) -> bool {
    inst.storages.iter().any(|s| rect_intersects(r, &s.rect))
}

fn sweep_rect(a: Rect, b: Rect) -> Rect {
    Rect {
        x1: a.x1.min(b.x1),
        y1: a.y1.min(b.y1),
        x2: a.x2.max(b.x2),
        y2: a.y2.max(b.y2),
    }
}

fn in_yard(inst: &Instance, bl: Point, dir: Direction) -> bool {
    if let Some(yard) = inst.yard_rect {
        let r = carrier_rect(bl, dir);
        rect_intersects(&r, &yard)
    } else {
        false
    }
}

// -------------------- Output cleanup: merge consecutive moves --------------------

fn compress_moves(cmds: Vec<Command>) -> Vec<Command> {
    let mut out: Vec<Command> = Vec::new();

    for cmd in cmds {
        match (out.last_mut(), &cmd) {
            (Some(Command::Move { t: t0, k: k0 }), Command::Move { t: t1, k: k1 }) => {
                let end_t0 = *t0 + k0.abs();
                if end_t0 == *t1 && k0.signum() == k1.signum() {
                    *k0 += *k1;
                    continue;
                }
                out.push(cmd);
            }
            _ => out.push(cmd),
        }
    }

    out
}

// -------------------- Static validity (map, yard, storages, dispatch) --------------------

fn is_valid_pos(inst: &Instance, bl: Point, dir: Direction) -> bool {
    let r = carrier_rect(bl, dir);

    // 1) Map bounds
    let map_limit = Rect { x1: 0, y1: 0, x2: inst.width - 1, y2: inst.height - 1 };
    if !rect_within(&r, &map_limit) {
        return false;
    }

    // 1.5) Yard constraint: inside yard you must be vertical (Up/Down)
    if in_yard(inst, bl, dir) {
        match dir {
            Direction::Up | Direction::Down => {}
            Direction::Left | Direction::Right => return false,
        }
    }

    // 2) Storages are obstacles, except "straddle" when vertical and aligned (x1 = storage.x1 - 1)
    let w = r.x2 - r.x1 + 1;
    let is_carrier_vert = w == 4;

    for s in &inst.storages {
        if rect_intersects(&r, &s.rect) {
            if is_carrier_vert && r.x1 == s.rect.x1 - 1 {
                continue; // allowed straddle lane
            }
            return false;
        }
    }

    // 3) Dispatch rectangles: if vertical, cannot be on top of dispatch
    if is_carrier_vert {
        for d in &inst.dispatches {
            if rect_intersects(&r, &d.rect) {
                return false;
            }
        }
    }

    true
}

/// Public wrapper for static pose validity (map + yard + storages + dispatch).
pub fn is_valid_pose(inst: &Instance, bl: Point, dir: Direction) -> bool {
    is_valid_pos(inst, bl, dir)
}

// -------------------- A* in space-time --------------------

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct State {
    x: i32,
    y: i32,
    dir: Direction,
    time: i32,
}

#[derive(Clone, Eq, PartialEq)]
struct Node {
    cost: i32,      // g
    heuristic: i32, // h
    state: State,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Action {
    Move(i32),
    Turn(Direction),
    Wait,
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        (other.cost + other.heuristic).cmp(&(self.cost + self.heuristic))
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn manhattan(s: &State, tx: i32, ty: i32) -> i32 {
    (s.x - tx).abs() + (s.y - ty).abs()
}

fn reconstruct_path(
    came_from: &HashMap<State, (State, Action)>,
    current: State,
    start: State,
) -> Vec<Command> {
    let mut path_cmds = Vec::new();
    let mut curr = current;

    while curr != start {
        if let Some((prev, act)) = came_from.get(&curr) {
            let t = prev.time;
            match act {
                Action::Move(k) => path_cmds.push(Command::Move { t, k: *k }),
                Action::Turn(d) => path_cmds.push(Command::Face { t, dir: *d }),
                Action::Wait => {}
            }
            curr = *prev;
        } else {
            break;
        }
    }

    path_cmds.reverse();
    compress_moves(path_cmds)
}

fn run_a_star(
    inst: &Instance,
    start_c: &CarrierState,
    target_bl: Point,
    target_dir: Direction,
    me: Id,
    res: &ReservationTable,
) -> Option<Vec<Command>> {
    let start_state = State {
        x: start_c.bl.x,
        y: start_c.bl.y,
        dir: start_c.dir,
        time: start_c.time,
    };

    let map_limit = Rect { x1: 0, y1: 0, x2: inst.width - 1, y2: inst.height - 1 };

    let mut open_set = BinaryHeap::new();
    let mut closed_set = HashSet::new();
    let mut came_from: HashMap<State, (State, Action)> = HashMap::new();

    open_set.push(Node {
        cost: 0,
        heuristic: manhattan(&start_state, target_bl.x, target_bl.y),
        state: start_state,
    });

    let mut iterations = 0;
    let max_iterations = 5_000_000;

    // Hard time horizon to keep space-time search finite. Increase if needed.
    let base_dist = (start_state.x - target_bl.x).abs() + (start_state.y - target_bl.y).abs();
    let max_time = start_state.time + base_dist * 8 + 5000;

    while let Some(current_node) = open_set.pop() {
        iterations += 1;
        if iterations > max_iterations {
            break;
        }

        let s = current_node.state;

        // Goal
        if s.x == target_bl.x && s.y == target_bl.y && s.dir == target_dir {
            return Some(reconstruct_path(&came_from, s, start_state));
        }

        if closed_set.contains(&s) {
            continue;
        }
        closed_set.insert(s);

        // Neighbors
        let mut neighbors: Vec<(State, Action)> = Vec::new();

        // 1) Wait
        neighbors.push((State { time: s.time + 1, ..s }, Action::Wait));

        // 2) Move (forward / backward)
        let (dx, dy) = match s.dir {
            Direction::Up => (0, 1),
            Direction::Down => (0, -1),
            Direction::Left => (-1, 0),
            Direction::Right => (1, 0),
        };
        neighbors.push((State { x: s.x + dx, y: s.y + dy, time: s.time + 1, ..s }, Action::Move(1)));
        neighbors.push((State { x: s.x - dx, y: s.y - dy, time: s.time + 1, ..s }, Action::Move(-1)));

        // 3) Turn (left / right)
        let center2 = center2_from_bl(Point { x: s.x, y: s.y }, s.dir);
        let rots = [
            match s.dir { Direction::Up=>Direction::Left,  Direction::Left=>Direction::Down, Direction::Down=>Direction::Right, Direction::Right=>Direction::Up },
            match s.dir { Direction::Up=>Direction::Right, Direction::Right=>Direction::Down,Direction::Down=>Direction::Left,  Direction::Left=>Direction::Up },
        ];
        for new_dir in rots {
            let new_bl = bl_from_center2(center2, new_dir);
            neighbors.push((
                State { x: new_bl.x, y: new_bl.y, dir: new_dir, time: s.time + 1 },
                Action::Turn(new_dir),
            ));
        }

        // Process Neighbors
        for (next_s, act) in neighbors {
            if closed_set.contains(&next_s) {
                continue;
            }

            // --- IMPORTANT: rotation "sweep" check (checker blocks if swept area hits storage) ---
            if let Action::Turn(_) = act {
                let before = carrier_rect(Point { x: s.x, y: s.y }, s.dir);
                let after  = carrier_rect(Point { x: next_s.x, y: next_s.y }, next_s.dir);
                let swept  = sweep_rect(before, after);

                // keep swept within map too (conservative)
                if !rect_within(&swept, &map_limit) {
                    continue;
                }
                if intersects_any_storage(inst, &swept) {
                    continue;
                }
            }

            if next_s.time > max_time {
                continue;
            }

            // 1) Static validity at target pose
            if !is_valid_pos(inst, Point { x: next_s.x, y: next_s.y }, next_s.dir) {
                continue;
            }

            // 2) Dynamic (time) collision check
            let r = carrier_rect(Point { x: next_s.x, y: next_s.y }, next_s.dir);
            if !res.is_free(next_s.time, me, &r) {
                continue;
            }

            // Store predecessor (first time)
            if !came_from.contains_key(&next_s) {
                came_from.insert(next_s, (s, act));
                let g = current_node.cost + 1;
                let h = manhattan(&next_s, target_bl.x, target_bl.y);
                open_set.push(Node { cost: g, heuristic: h, state: next_s });
            }
        }
    }

    None
}

// -------------------- Public interface: execute planned commands and reserve --------------------

pub fn go_to_pose(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
    res: &mut ReservationTable,
) {
    if let Some(new_cmds) = run_a_star(inst, c, target_bl, target_dir, c.id, res) {
        for cmd in new_cmds {
            cmds.push(cmd.clone());

            // Sync time with command start (implicit waits)
            let cmd_t = match &cmd {
                Command::Move { t, .. } => *t,
                Command::Face { t, .. } => *t,
                Command::Load { t } => *t,
                Command::Unload { t } => *t,
            };

            while c.time < cmd_t {
                c.time += 1;
                res.reserve(c.time, c.id, carrier_rect(c.bl, c.dir));
            }

            match cmd {
                Command::Move { t: _, k } => {
                    let (dx, dy) = match c.dir {
                        Direction::Up => (0, 1),
                        Direction::Down => (0, -1),
                        Direction::Left => (-1, 0),
                        Direction::Right => (1, 0),
                    };

                    let step = k.signum();
                    for _ in 0..k.abs() {
                        let prev = carrier_rect(c.bl, c.dir);
                        c.time += 1;
                        c.bl.x += dx * step;
                        c.bl.y += dy * step;
                        let now = carrier_rect(c.bl, c.dir);

                        // Conservative anti-swap / anti-crossing reservation:
                        // reserve swept area for THIS carrier at this second.
                        res.reserve(c.time, c.id, sweep_rect(prev, now));
                    }
                }

                Command::Face { t: _, dir } => {
                    let prev = carrier_rect(c.bl, c.dir);

                    c.time += 1;
                    let center2 = center2_from_bl(c.bl, c.dir);
                    c.dir = dir;
                    c.bl = bl_from_center2(center2, dir);

                    let now = carrier_rect(c.bl, c.dir);

                    // Reserve swept area during rotation second (conservative)
                    res.reserve(c.time, c.id, sweep_rect(prev, now));
                }

                _ => {
                    // Load/Unload handled in simple.rs
                }
            }
        }
    } else {
        // Fallback: if no path is found (often due to temporary space-time reservations),
        // try inserting idle time (implicit waiting) and re-run A* with a shifted timeline.
        let mut waited_total = 0;
        for attempt in 0..10 {
            let wait = (attempt + 1) * 10; // 10,20,...,100
            for _ in 0..wait {
                c.time += 1;
                res.reserve(c.time, c.id, carrier_rect(c.bl, c.dir));
            }
            waited_total += wait;

            if let Some(new_cmds) = run_a_star(inst, c, target_bl, target_dir, c.id, res) {
                // execute recursively but without infinite recursion: inline execution
                for cmd in new_cmds {
                    cmds.push(cmd.clone());
                    let cmd_t = match &cmd {
                        Command::Move { t, .. } => *t,
                        Command::Face { t, .. } => *t,
                        Command::Load { t } => *t,
                        Command::Unload { t } => *t,
                    };
                    while c.time < cmd_t {
                        c.time += 1;
                        res.reserve(c.time, c.id, carrier_rect(c.bl, c.dir));
                    }
                    match cmd {
                        Command::Move { t: _, k } => {
                            let (dx, dy) = match c.dir {
                                Direction::Up => (0, 1),
                                Direction::Down => (0, -1),
                                Direction::Left => (-1, 0),
                                Direction::Right => (1, 0),
                            };
                            let step = k.signum();
                            for _ in 0..k.abs() {
                                let prev = carrier_rect(c.bl, c.dir);
                                c.time += 1;
                                c.bl.x += dx * step;
                                c.bl.y += dy * step;
                                let now = carrier_rect(c.bl, c.dir);
                                res.reserve(c.time, c.id, sweep_rect(prev, now));
                            }
                        }
                        Command::Face { t: _, dir } => {
                            let prev = carrier_rect(c.bl, c.dir);
                            c.time += 1;
                            let center2 = center2_from_bl(c.bl, c.dir);
                            c.dir = dir;
                            c.bl = bl_from_center2(center2, dir);
                            let now = carrier_rect(c.bl, c.dir);
                            res.reserve(c.time, c.id, sweep_rect(prev, now));
                        }
                        _ => {}
                    }
                }
                return;
            }
        }

        panic!(
            "A* No Path Found after waiting {}s: carrier {} time {} -> target bl=({},{}), dir={:?}",
            waited_total, c.id, c.time, target_bl.x, target_bl.y, target_dir
        );
    }
}
