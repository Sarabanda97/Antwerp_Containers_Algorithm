use std::collections::{BinaryHeap, HashMap, HashSet};
use std::cmp::Ordering;

use crate::model::{Direction, Instance, Point, Rect};
use crate::state::CarrierState;


pub struct ReservationTable {
    occupied: HashMap<i32, Vec<Rect>>,
}

impl ReservationTable {
    pub fn new() -> Self {
        Self { occupied: HashMap::new() }
    }

    pub fn is_free(&self, t: i32, r: &Rect) -> bool {
        if let Some(obstacles) = self.occupied.get(&t) {
            for obs in obstacles {
                if rect_intersects(r, obs) {
                    return false;
                }
            }
        }
        true
    }

    pub fn reserve(&mut self, t: i32, r: Rect) {
        self.occupied.entry(t).or_default().push(r);
    }

    /// Reserve cell with temporal safety buffer to prevent tailgating collisions.
    /// Linger extends reservation for N timesteps to ensure previous carrier clears the area.
    pub fn reserve_with_linger(&mut self, t: i32, r: Rect, linger: i32) {
        for dt in 0..=linger {
            self.reserve(t + dt, r);
        }
    }
}

#[derive(Clone, Debug)]
pub enum Command {
    Move   { t: i32, k: i32 },
    Face   { t: i32, dir: Direction },
    Load   { t: i32 },
    Unload { t: i32 },
}



// Carrier dimensions: 4×8 when vertical (Up/Down), 8×4 when horizontal (Left/Right)
const SHORT: i32 = 4;
const LONG:  i32 = 8;

fn dims(dir: Direction) -> (i32, i32) {
    match dir {
        Direction::Up | Direction::Down => (SHORT, LONG),
        Direction::Left | Direction::Right => (LONG, SHORT),
    }
}

/// Convert bottom-left to center in doubled coordinates (avoids floats during rotation).
/// Center formula: 2*center = 2*bl + (w-1, h-1)
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



/// Check static validity: map bounds, yard constraint (vertical only), storage/dispatch obstacles.
/// Allows "straddle" positioning for vertical carriers at x = storage.x - 1.
fn is_valid_pos(inst: &Instance, bl: Point, dir: Direction) -> bool {
    let r = carrier_rect(bl, dir);

    // Map boundary check
    let map_limit = Rect { x1: 0, y1: 0, x2: inst.width - 1, y2: inst.height - 1 };
    if !rect_within(&r, &map_limit) {
        return false;
    }

    // Yard constraint: only vertical carriers allowed inside yard
    if in_yard(inst, bl, dir) {
        match dir {
            Direction::Up | Direction::Down => {}
            Direction::Left | Direction::Right => return false,
        }
    }

    let w = r.x2 - r.x1 + 1;
    let is_carrier_vert = w == 4;

    // Storage collision check with straddle exception
    for s in &inst.storages {
        if rect_intersects(&r, &s.rect) {
            if is_carrier_vert && r.x1 == s.rect.x1 - 1 {
                continue; // Allow straddling storage from staging lane
            }
            return false;
        }
    }

    if is_carrier_vert {
        for d in &inst.dispatches {
            if rect_intersects(&r, &d.rect) {
                return false;
            }
        }
    }

    true
}

pub fn is_valid_pose(inst: &Instance, bl: Point, dir: Direction) -> bool {
    is_valid_pos(inst, bl, dir)
}


// Space-time A* state: (x, y, direction, time)
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
struct State {
    x: i32,
    y: i32,
    dir: Direction,
    time: i32,
}

#[derive(Clone, Eq, PartialEq)]
struct Node {
    cost: i32,      // g-cost (actual path length)
    heuristic: i32, // h-cost (Manhattan distance to goal) 
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

    // Time horizon: base distance * 32 (detour allowance) + 50k buffer
    let base_dist = (start_state.x - target_bl.x).abs() + (start_state.y - target_bl.y).abs();
    let max_time = start_state.time + base_dist * 32 + 50000;

    while let Some(current_node) = open_set.pop() {
        iterations += 1;
        if iterations > max_iterations {
            break;
        }

        let s = current_node.state;

        if s.x == target_bl.x && s.y == target_bl.y && s.dir == target_dir {
            return Some(reconstruct_path(&came_from, s, start_state));
        }

        if closed_set.contains(&s) {
            continue;
        }
        closed_set.insert(s);

        // Generate neighbor states: wait, move forward/backward, rotate left/right
        let mut neighbors: Vec<(State, Action)> = Vec::new();

        neighbors.push((State { time: s.time + 1, ..s }, Action::Wait));

        let (dx, dy) = match s.dir {
            Direction::Up => (0, 1),
            Direction::Down => (0, -1),
            Direction::Left => (-1, 0),
            Direction::Right => (1, 0),
        };
        neighbors.push((State { x: s.x + dx, y: s.y + dy, time: s.time + 1, ..s }, Action::Move(1)));
        neighbors.push((State { x: s.x - dx, y: s.y - dy, time: s.time + 1, ..s }, Action::Move(-1)));

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

        for (next_s, act) in neighbors {
            if closed_set.contains(&next_s) {
                continue;
            }

            let current_rect = carrier_rect(Point { x: s.x, y: s.y }, s.dir);
            let next_rect = carrier_rect(Point { x: next_s.x, y: next_s.y }, next_s.dir);

            // Swept-area collision checks prevent head-on swaps and rotation conflicts
            match act {
                Action::Turn(_) => {
                    let swept = sweep_rect(current_rect, next_rect);
                    if !rect_within(&swept, &map_limit) {
                        continue;
                    }
                    if intersects_any_storage(inst, &swept) {
                        continue;
                    }
                }
                Action::Move(_) => {
                    // Check current position is free to prevent overlaps during movement
                    if !res.is_free(next_s.time, &current_rect) {
                        continue;
                    }
                }
                Action::Wait => {}
            }

            if next_s.time > max_time {
                continue;
            }

            // Static validity (map/obstacles) and dynamic (time-based reservations)
            if !is_valid_pos(inst, Point { x: next_s.x, y: next_s.y }, next_s.dir) {
                continue;
            }

            let r = carrier_rect(Point { x: next_s.x, y: next_s.y }, next_s.dir);
            if !res.is_free(next_s.time, &r) {
                continue;
            }

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


pub fn go_to_pose(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
    res: &mut ReservationTable,
) {
    if let Some(new_cmds) = run_a_star(inst, c, target_bl, target_dir, res) {
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
                res.reserve_with_linger(c.time, carrier_rect(c.bl, c.dir), 4);
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
                        // Reserve both source and destination to prevent head-on conflicts
                        let prev = carrier_rect(c.bl, c.dir);
                        c.time += 1;
                        c.bl.x += dx * step;
                        c.bl.y += dy * step;
                        let now = carrier_rect(c.bl, c.dir);

                        res.reserve_with_linger(c.time, prev, 4);
                        res.reserve_with_linger(c.time, now, 4);
                    }
                }

                Command::Face { t: _, dir } => {
                    let prev = carrier_rect(c.bl, c.dir);

                    // Rotate around center point (using doubled coordinates)
                    c.time += 1;
                    let center2 = center2_from_bl(c.bl, c.dir);
                    c.dir = dir;
                    c.bl = bl_from_center2(center2, dir);

                    let now = carrier_rect(c.bl, c.dir);

                    res.reserve_with_linger(c.time, prev, 4);
                    res.reserve_with_linger(c.time, now, 4);
                }

                _ => {
                }
            }
        }
    } else {
        // Fallback: retry with incremental wait times (10, 20, ..., 500 ticks)
        // Speculative waits don't reserve to avoid self-blocking
        let mut waited_total = 0;
        for attempt in 0..50 {
            let wait = (attempt + 1) * 10;
            c.time += wait;
            waited_total += wait;

            if let Some(new_cmds) = run_a_star(inst, c, target_bl, target_dir, res) {
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
                        res.reserve_with_linger(c.time, carrier_rect(c.bl, c.dir), 4);
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
                                res.reserve_with_linger(c.time, prev, 4);
                                res.reserve_with_linger(c.time, now, 4);
                            }
                        }
                        Command::Face { t: _, dir } => {
                            let prev = carrier_rect(c.bl, c.dir);
                            c.time += 1;
                            let center2 = center2_from_bl(c.bl, c.dir);
                            c.dir = dir;
                            c.bl = bl_from_center2(center2, dir);
                            let now = carrier_rect(c.bl, c.dir);
                            res.reserve_with_linger(c.time, prev, 4);
                            res.reserve_with_linger(c.time, now, 4);
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
