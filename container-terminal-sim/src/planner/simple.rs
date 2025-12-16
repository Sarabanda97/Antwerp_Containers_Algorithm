use std::collections::HashMap;

use crate::model::{Demand, Direction, Id, Instance, Point, Rect};
use crate::planner::path::{carrier_rect, go_to_pose, is_valid_pose, Command, ReservationTable};
use crate::state::CarrierState;

#[derive(Clone, Copy, Debug)]
struct Pose {
    bl: Point,
    dir: Direction,
}

#[derive(Clone, Debug)]
enum ContainerLocation {
    Storage { storage_id: Id, depth: usize },
    Dispatch { dispatch_id: Id },
    OnCarrier { carrier_id: Id },
}

struct PlanningContext<'a> {
    inst: &'a Instance,
    c: &'a mut CarrierState,
    cmds: &'a mut Vec<Command>,
    storage_stacks: &'a mut Vec<Vec<Id>>,
    locs: &'a mut HashMap<Id, ContainerLocation>,
    dispatch_containers: &'a mut HashMap<Id, Vec<Id>>,
    storage_idx: &'a HashMap<Id, usize>,
    dispatch_idx: &'a HashMap<Id, usize>,
    res: &'a mut ReservationTable,
}

fn rect_intersects(a: &Rect, b: &Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}

fn rect_contains(outer: &Rect, inner: &Rect) -> bool {
    inner.x1 >= outer.x1 && inner.y1 >= outer.y1 && inner.x2 <= outer.x2 && inner.y2 <= outer.y2
}

/// Scan the map for "parking" poses that are:
/// - valid by the same static validator used by A*
/// - outside yard (reject if fully contained in yard_rect)
/// - outside all crane rectangles and dispatch rectangles
/// - non-overlapping between themselves
///
/// We bias towards the periphery (smaller score = closer to border).
fn compute_parking_poses(inst: &Instance, n: usize) -> Vec<Pose> {
    let mut candidates: Vec<(i32, Pose, Rect)> = Vec::new();

    // Coarse scan for speed.
    let step: usize = 4;
    let w: usize = inst.width.max(0) as usize;
    let h: usize = inst.height.max(0) as usize;

    // Prefer poses that are easy to reach / align (Down or Right first).
    let dirs = [Direction::Down, Direction::Right, Direction::Up, Direction::Left];

    for yy in (0..h).step_by(step) {
        for xx in (0..w).step_by(step) {
            let bl = Point {
                x: xx as i32,
                y: yy as i32,
            };

            // Choose the first direction that is valid (preference order above).
            let mut chosen: Option<(Pose, Rect)> = None;
            for &dir in &dirs {
                if !is_valid_pose(inst, bl, dir) {
                    continue;
                }
                let r = carrier_rect(bl, dir);

                // Keep parking out of the yard rectangle (traffic zone).
                if let Some(yard) = inst.yard_rect {
                    if rect_contains(&yard, &r) {
                        continue;
                    }
                }

                // Avoid cranes and dispatches.
                if inst.cranes.iter().any(|c| rect_intersects(&r, &c.rect)) {
                    continue;
                }
                if inst.dispatches.iter().any(|d| rect_intersects(&r, &d.rect)) {
                    continue;
                }

                // Also avoid sitting on top of storages (even if "straddle" would allow).
                if inst.storages.iter().any(|s| rect_intersects(&r, &s.rect)) {
                    continue;
                }

                chosen = Some((Pose { bl, dir }, r));
                break;
            }

            if let Some((pose, r)) = chosen {
                // Peripheral bias: smaller = closer to border.
                let dx = (pose.bl.x).min((inst.width - 1) - pose.bl.x);
                let dy = (pose.bl.y).min((inst.height - 1) - pose.bl.y);
                let score = dx + dy;
                candidates.push((score, pose, r));
            }
        }
    }

    // Sort best first (closer to the border).
    candidates.sort_by_key(|(score, _, _)| *score);

    // Greedy selection with non-overlap constraint.
    let mut chosen: Vec<(Pose, Rect)> = Vec::new();
    'outer: for (_, pose, r) in candidates {
        for (_, rr) in &chosen {
            if rect_intersects(&r, rr) {
                continue 'outer;
            }
        }
        chosen.push((pose, r));
        if chosen.len() >= n {
            break;
        }
    }

    chosen.into_iter().map(|(p, _)| p).collect()
}

impl<'a> PlanningContext<'a> {
    fn goto_staging(&mut self, target_bl: Point, target_dir: Direction) {
        go_to_pose(self.inst, self.c, target_bl, target_dir, self.cmds, self.res);
    }

    fn reserve_idle(&mut self, dt: i32) {
        for _ in 0..dt {
            self.c.time += 1;
            self.res.reserve(self.c.time, self.c.id, carrier_rect(self.c.bl, self.c.dir));
        }
    }

    fn locate_and_register_in_stacks(&mut self, cid: Id) -> Option<ContainerLocation> {
        for (s_idx, stack) in self.storage_stacks.iter().enumerate() {
            if let Some(depth) = stack.iter().position(|&x| x == cid) {
                let sid = self.inst.storages[s_idx].id;
                let loc = ContainerLocation::Storage { storage_id: sid, depth };
                self.locs.insert(cid, loc.clone());
                return Some(loc);
            }
        }
        None
    }

    fn load_from_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).expect("dispatch_idx missing");
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.expect("Dispatch staging_bl missing");
        let dir = disp.staging_dir.expect("Dispatch staging_dir missing");
        self.goto_staging(bl, dir);

        if let Some(vec) = self.dispatch_containers.get_mut(&dispatch_id) {
            if let Some(pos) = vec.iter().position(|&x| x == container_id) {
                vec.remove(pos);
            }
        }

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;

        self.c.carrying = Some(container_id);
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });

        self.res.reserve(self.c.time, self.c.id, carrier_rect(self.c.bl, self.c.dir));
    }

    fn unload_to_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).expect("dispatch_idx missing");
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.expect("Dispatch staging_bl missing");
        let dir = disp.staging_dir.expect("Dispatch staging_dir missing");
        self.goto_staging(bl, dir);

        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;

        self.c.carrying = None;

        self.dispatch_containers.entry(dispatch_id).or_default().push(container_id);
        self.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });

        self.res.reserve(self.c.time, self.c.id, carrier_rect(self.c.bl, self.c.dir));
    }

    fn load_from_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).expect("storage_idx missing");
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.expect("Storage staging_bl missing");
        let dir = stor.staging_dir.expect("Storage staging_dir missing");
        self.goto_staging(bl, dir);

        let stack = &mut self.storage_stacks[s_idx];
        let top = *stack.last().expect("Storage vazio");
        if top != container_id {
            panic!(
                "Trying to LOAD container {} but top is {} in storage {} (ensure_container_accessible failed?)",
                container_id, top, storage_id
            );
        }
        stack.pop();

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;

        self.c.carrying = Some(container_id);
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });

        self.res.reserve(self.c.time, self.c.id, carrier_rect(self.c.bl, self.c.dir));
    }

    fn unload_to_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).expect("storage_idx missing");
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.expect("Storage staging_bl missing");
        let dir = stor.staging_dir.expect("Storage staging_dir missing");
        self.goto_staging(bl, dir);

        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;

        self.c.carrying = None;

        let stack = &mut self.storage_stacks[s_idx];
        stack.push(container_id);
        let depth = stack.len() - 1;

        self.locs.insert(container_id, ContainerLocation::Storage { storage_id, depth });

        self.res.reserve(self.c.time, self.c.id, carrier_rect(self.c.bl, self.c.dir));
    }

    fn find_best_temp_storage(&self, exclude_id: Id) -> Option<Id> {
        let current_pos = self.c.bl;
        let mut best_id: Option<Id> = None;
        let mut best_score = i32::MAX;

        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id {
                continue;
            }
            if self.storage_stacks[i].len() >= 2 {
                continue;
            }
            if let Some(target_bl) = s.staging_bl {
                let dist = (target_bl.x - current_pos.x).abs() + (target_bl.y - current_pos.y).abs();
                let penalty = if self.storage_stacks[i].len() > 0 { 200 } else { 0 };
                let score = dist + penalty;
                if score < best_score {
                    best_score = score;
                    best_id = Some(s.id);
                }
            }
        }
        best_id
    }

    fn ensure_container_accessible(&mut self, storage_id: Id, target_cid: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).expect("storage_idx missing");

        loop {
            let stack_len = self.storage_stacks[s_idx].len();
            if stack_len == 0 {
                panic!("Storage {} empty while trying to access container {}", storage_id, target_cid);
            }
            let top_cid = self.storage_stacks[s_idx][stack_len - 1];
            if top_cid == target_cid {
                return;
            }

            let temp_storage_id = match self.find_best_temp_storage(storage_id) {
                Some(id) => id,
                None => {
                    // Yard "full" fallback: this should not happen on basic instances,
                    // but we avoid a hard panic here to keep planning alive.
                    // Strategy: pick any other storage (even if full) and just use it;
                    // if it's full, the subsequent unload will fail and show a clearer error.
                    self.inst
                        .storages
                        .iter()
                        .find(|s| s.id != storage_id)
                        .map(|s| s.id)
                        .expect("No alternative storage exists")
                }
            };

            self.load_from_storage(storage_id, top_cid);
            self.unload_to_storage(temp_storage_id, top_cid);
        }
    }
}

/// Pick a waiting pose outside the crane section.
fn choose_outside_crane_pose(inst: &Instance, crane_id: Id, from: &CarrierState) -> (Point, Direction) {
    let crane = inst.cranes.iter().find(|c| c.id == crane_id).expect("crane not found");
    let cr = crane.rect;

    let mut candidates: Vec<(Point, Direction)> = Vec::new();

    // Above / below: vertical poses (4x8)
    for &x in &[cr.x1, cr.x2 - 3, cr.x1 - 4, cr.x2 + 1] {
        candidates.push((Point { x, y: cr.y2 + 1 }, Direction::Up));
        candidates.push((Point { x, y: cr.y1 - 8 }, Direction::Down));
    }
    // Left / right: horizontal poses (8x4)
    for &y in &[cr.y1, cr.y2 - 3, cr.y1 - 4, cr.y2 + 1] {
        candidates.push((Point { x: cr.x1 - 8, y }, Direction::Left));
        candidates.push((Point { x: cr.x2 + 1, y }, Direction::Right));
    }

    let mut best: Option<(Point, Direction, i32)> = None;

    for (bl, dir) in candidates {
        if !is_valid_pose(inst, bl, dir) {
            continue;
        }
        let rr = carrier_rect(bl, dir);
        if rect_intersects(&rr, &cr) {
            continue;
        }
        let dist = (bl.x - from.bl.x).abs() + (bl.y - from.bl.y).abs();
        match best {
            None => best = Some((bl, dir, dist)),
            Some((_, _, bd)) if dist < bd => best = Some((bl, dir, dist)),
            _ => {}
        }
    }

    if let Some((bl, dir, _)) = best {
        return (bl, dir);
    }

    // Worst-case fallback: just step left of crane; A* may still find a valid nearby pose.
    (Point { x: cr.x1 - 8, y: cr.y1 }, Direction::Left)
}

fn ensure_outside_crane_and_wait(
    inst: &Instance,
    crane_id: Id,
    ctx: &mut PlanningContext<'_>,
    dt: i32,
) {
    let crane = inst.cranes.iter().find(|c| c.id == crane_id).expect("crane not found");
    let cr = crane.rect;
    let rr = carrier_rect(ctx.c.bl, ctx.c.dir);

    if rect_intersects(&rr, &cr) {
        let (w_bl, w_dir) = choose_outside_crane_pose(inst, crane_id, ctx.c);
        ctx.goto_staging(w_bl, w_dir);
    }

    // Ensure outside even if we're near boundary
    let rr2 = carrier_rect(ctx.c.bl, ctx.c.dir);
    if rect_intersects(&rr2, &cr) {
        let (w_bl, w_dir) = choose_outside_crane_pose(inst, crane_id, ctx.c);
        ctx.goto_staging(w_bl, w_dir);
    }

    ctx.reserve_idle(dt);
}

fn go_to_parking(
    inst: &Instance,
    carrier: &mut CarrierState,
    cmds: &mut Vec<Command>,
    res: &mut ReservationTable,
    pose: Pose,
) {
    // Move to the parking pose (A* + reservations).
    go_to_pose(inst, carrier, pose.bl, pose.dir, cmds, res);

    // Ensure an explicit reservation at the final parking time.
    // NOTE: ReservationTable already keeps the last occupancy of each carrier and
    // treats it as persistent for future times ("tail"), so parked carriers become
    // permanent obstacles until they move again.
    res.reserve(carrier.time, carrier.id, carrier_rect(carrier.bl, carrier.dir));
}

pub fn plan_all_demands_multi(inst: &Instance) -> Vec<(Id, Vec<Command>)> {
    let mut storage_stacks = inst.storage_stacks.clone();
    let mut dispatch_containers: HashMap<Id, Vec<Id>> = HashMap::new();
    let mut locs: HashMap<Id, ContainerLocation> = HashMap::new();
    let mut reservation_table = ReservationTable::new();

    for d in &inst.dispatches {
        dispatch_containers.insert(d.id, Vec::new());
    }

    for (s_idx, stack) in storage_stacks.iter().enumerate() {
        let sid = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            locs.insert(cid, ContainerLocation::Storage { storage_id: sid, depth });
        }
    }

    let mut storage_idx: HashMap<Id, usize> = HashMap::new();
    for (i, s) in inst.storages.iter().enumerate() {
        storage_idx.insert(s.id, i);
    }
    let mut dispatch_idx: HashMap<Id, usize> = HashMap::new();
    for (i, d) in inst.dispatches.iter().enumerate() {
        dispatch_idx.insert(d.id, i);
    }

    // Carrier states + per-carrier command buffers
    let mut carriers: Vec<CarrierState> = inst
        .carriers
        .iter()
        .map(|c| CarrierState {
            id: c.id,
            bl: c.bl,
            dir: c.dir,
            carrying: None,
            time: 0,
        })
        .collect();

    let mut cmds_by_carrier: HashMap<Id, Vec<Command>> = HashMap::new();
    for c in &carriers {
        cmds_by_carrier.insert(c.id, Vec::new());
        reservation_table.reserve(0, c.id, carrier_rect(c.bl, c.dir));
    }

    // Demands per crane in order
    let mut demands_per_crane: HashMap<Id, Vec<Demand>> = HashMap::new();
    if !inst.ships.is_empty() {
        for ship in &inst.ships {
            if let Some(crane_id) = ship.crane_id {
                demands_per_crane.entry(crane_id).or_default().extend(ship.operations.clone());
            }
        }
    } else {
        // Fallback (no ship blocks)
        demands_per_crane.entry(0).or_default().extend(inst.demands.clone());
    }

    // ---------------- Dynamic carrier pool per crane (min 2, scale to 3) ----------------

    let min_per_crane: usize = 2;
    let max_per_crane: usize = 3;
    let backlog_threshold: usize = 10;

    // Stable crane iteration order.
    let mut crane_ids: Vec<Id> = inst.cranes.iter().map(|c| c.id).collect();
    crane_ids.sort();

    // Carrier id -> index in `carriers` vec.
    let mut carrier_idx_by_id: HashMap<Id, usize> = HashMap::new();
    for (i, c) in carriers.iter().enumerate() {
        carrier_idx_by_id.insert(c.id, i);
    }

    // Parking poses (one per carrier id, deterministically).
    let n_parking = inst.carriers.len().max(6);
    let parking_poses = compute_parking_poses(inst, n_parking);
    let mut parking_assign: HashMap<Id, Pose> = HashMap::new();
    for (i, c) in inst.carriers.iter().enumerate() {
        if let Some(p) = parking_poses.get(i) {
            parking_assign.insert(c.id, *p);
        }
    }

    // Initial pool assignment: distribute carriers by id across cranes, ensuring
    // at least `min_per_crane` active per crane when possible.
    let mut carrier_ids: Vec<Id> = carriers.iter().map(|c| c.id).collect();
    carrier_ids.sort();

    let mut active_by_crane: HashMap<Id, Vec<Id>> = HashMap::new();
    for &crane_id in &crane_ids {
        active_by_crane.insert(crane_id, Vec::new());
    }

    let mut idle_pool: Vec<Id> = Vec::new();
    let mut it = carrier_ids.into_iter();
    for &crane_id in &crane_ids {
        for _ in 0..min_per_crane {
            if let Some(cid) = it.next() {
                active_by_crane.get_mut(&crane_id).unwrap().push(cid);
            }
        }
    }
    while let Some(cid) = it.next() {
        idle_pool.push(cid);
    }

    // Park carriers that start as idle (move them to a safe peripheral pose).
    for &cid in &idle_pool {
        let idx = *carrier_idx_by_id.get(&cid).expect("missing carrier index");
        let pose = *parking_assign
            .get(&cid)
            .unwrap_or_else(|| parking_poses.first().expect("no parking poses computed"));
        let cmds = cmds_by_carrier.get_mut(&cid).unwrap();
        go_to_parking(inst, &mut carriers[idx], cmds, &mut reservation_table, pose);
    }

    // Crane time synchronization
    let mut crane_time: HashMap<Id, i32> = HashMap::new();
    for &crane_id in &crane_ids {
        crane_time.insert(crane_id, 0);
    }

    // Fairness tracking
    let mut jobs_done: HashMap<Id, i32> = HashMap::new();
    for c in &carriers {
        jobs_done.insert(c.id, 0);
    }
    let mut last_used: HashMap<Id, Id> = HashMap::new();

    // Per-crane demand pointer (default 0)
    let mut ptr: HashMap<Id, usize> = HashMap::new();
    for &crane_id in &crane_ids {
        ptr.insert(crane_id, 0);
    }

    // Estimate the next required pose for a demand (only for scheduling heuristics).
    fn estimate_target_pose(
        inst: &Instance,
        demand: &Demand,
        locs: &HashMap<Id, ContainerLocation>,
        storage_idx: &HashMap<Id, usize>,
        dispatch_idx: &HashMap<Id, usize>,
    ) -> Pose {
        match *demand {
            Demand::Unload { dispatch_id, .. } => {
                let di = *dispatch_idx.get(&dispatch_id).unwrap();
                let d = &inst.dispatches[di];
                Pose { bl: d.staging_bl.unwrap(), dir: d.staging_dir.unwrap() }
            }
            Demand::Load { dispatch_id, container_id } => {
                if let Some(loc) = locs.get(&container_id) {
                    match *loc {
                        ContainerLocation::Storage { storage_id, .. } => {
                            let si = *storage_idx.get(&storage_id).unwrap();
                            let s = &inst.storages[si];
                            return Pose { bl: s.staging_bl.unwrap(), dir: s.staging_dir.unwrap() };
                        }
                        ContainerLocation::Dispatch { dispatch_id: d2 } => {
                            let di = *dispatch_idx.get(&d2).unwrap();
                            let d = &inst.dispatches[di];
                            return Pose { bl: d.staging_bl.unwrap(), dir: d.staging_dir.unwrap() };
                        }
                        ContainerLocation::OnCarrier { .. } => {}
                    }
                }
                // Fallback: target the dispatch.
                let di = *dispatch_idx.get(&dispatch_id).unwrap();
                let d = &inst.dispatches[di];
                Pose { bl: d.staging_bl.unwrap(), dir: d.staging_dir.unwrap() }
            }
        }
    }

    // Main scheduling loop: for each crane, always keep at least 2 carriers active (when possible),
    // and scale to 3 if backlog is large and there is an idle carrier available.
    loop {
        let mut any_left = false;

        for &crane_id in &crane_ids {
            let demands = match demands_per_crane.get(&crane_id) {
                Some(v) if !v.is_empty() => v,
                _ => continue,
            };

            let i = *ptr.get(&crane_id).unwrap_or(&0);
            if i >= demands.len() {
                continue;
            }
            any_left = true;

            // Scale up to 3 if needed.
            let remaining = demands.len() - i;
            {
                let active = active_by_crane.get_mut(&crane_id).unwrap();
                while remaining > backlog_threshold && active.len() < max_per_crane && !idle_pool.is_empty() {
                    let cid = idle_pool.pop().unwrap();
                    active.push(cid);
                }
            }

            let demand = demands[i].clone();
            let ready_t = *crane_time.get(&crane_id).unwrap_or(&0);
            let target = estimate_target_pose(inst, &demand, &locs, &storage_idx, &dispatch_idx);

            // Choose the best active carrier for this demand.
            let chosen_cid: Id = {
                let active = active_by_crane.get(&crane_id).unwrap();
                if active.is_empty() {
                    continue;
                }
                let mut best_cid = active[0];
                let mut best_score = i32::MAX;

                for &cid in active {
                    let idx = *carrier_idx_by_id.get(&cid).unwrap();
                    let c = &carriers[idx];

                    let mut sc = c.time.max(ready_t);
                    let dist = (c.bl.x - target.bl.x).abs() + (c.bl.y - target.bl.y).abs();
                    sc += dist;

                    // Fairness: avoid reusing the same carrier for consecutive operations on the same crane.
                    if last_used.get(&crane_id).copied() == Some(cid) {
                        sc += 20;
                    }

                    // Mild load-balancing penalty.
                    let done = *jobs_done.get(&cid).unwrap_or(&0);
                    sc += done * 2;

                    if sc < best_score {
                        best_score = sc;
                        best_cid = cid;
                    }
                }

                best_cid
            };

            // Execute demand with chosen carrier.
            {
                let chosen_idx = *carrier_idx_by_id.get(&chosen_cid).unwrap();
                let cmds = cmds_by_carrier.get_mut(&chosen_cid).unwrap();

                let mut ctx = PlanningContext {
                    inst,
                    c: &mut carriers[chosen_idx],
                    cmds,
                    storage_stacks: &mut storage_stacks,
                    locs: &mut locs,
                    dispatch_containers: &mut dispatch_containers,
                    storage_idx: &storage_idx,
                    dispatch_idx: &dispatch_idx,
                    res: &mut reservation_table,
                };

                // Sync with crane time
                if ctx.c.time < ready_t {
                    ctx.reserve_idle(ready_t - ctx.c.time);
                }

                match demand {
                    Demand::Unload { dispatch_id, container_id, storage_id } => {
                        // Ship unload only when carrier is OUTSIDE crane section
                        ensure_outside_crane_and_wait(inst, crane_id, &mut ctx, 1);

                        // Now container exists on dispatch (from ship)
                        ctx.dispatch_containers.entry(dispatch_id).or_default().push(container_id);
                        ctx.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });

                        // Carrier picks it up and stores it
                        ctx.load_from_dispatch(dispatch_id, container_id);
                        ctx.unload_to_storage(storage_id, container_id);
                    }
                    Demand::Load { dispatch_id, container_id } => {
                        // Find container
                        let loc = ctx
                            .locs
                            .get(&container_id)
                            .cloned()
                            .or_else(|| ctx.locate_and_register_in_stacks(container_id))
                            .unwrap_or_else(|| {
                                panic!(
                                    "LOAD requested for container {} but its location is unknown (not in stacks/dispatch)",
                                    container_id
                                )
                            });

                        match loc {
                            ContainerLocation::Storage { storage_id, .. } => {
                                ctx.ensure_container_accessible(storage_id, container_id);
                                ctx.load_from_storage(storage_id, container_id);
                            }
                            ContainerLocation::Dispatch { dispatch_id: d } => {
                                ctx.load_from_dispatch(d, container_id);
                            }
                            ContainerLocation::OnCarrier { .. } => {
                                panic!("Container {} already on a carrier while processing demand", container_id);
                            }
                        }

                        // Deliver to dispatch for ship
                        ctx.unload_to_dispatch(dispatch_id, container_id);

                        // Ship load only when carrier is OUTSIDE crane section
                        ensure_outside_crane_and_wait(inst, crane_id, &mut ctx, 1);

                        // After ship loads, container disappears from dispatch
                        if let Some(vec) = ctx.dispatch_containers.get_mut(&dispatch_id) {
                            if let Some(pos) = vec.iter().position(|&x| x == container_id) {
                                vec.remove(pos);
                            }
                        }
                        ctx.locs.remove(&container_id);
                    }
                }

                // Update crane time after operation
                crane_time.insert(crane_id, ctx.c.time);

                // Mark this carrier as used
                *jobs_done.entry(chosen_cid).or_insert(0) += 1;
                last_used.insert(crane_id, chosen_cid);
            }

            // Advance pointer
            ptr.insert(crane_id, i + 1);

            // Scale down: if backlog is now small, park extra carriers (keep at least min_per_crane).
            let after_i = *ptr.get(&crane_id).unwrap_or(&0);
            let remaining_after = demands.len().saturating_sub(after_i);
            if remaining_after <= backlog_threshold {
                // Collect carriers to park (drop to min_per_crane).
                let mut to_park: Vec<Id> = Vec::new();
                {
                    let active = active_by_crane.get_mut(&crane_id).unwrap();
                    while active.len() > min_per_crane {
                        if let Some(cid) = active.pop() {
                            to_park.push(cid);
                        }
                    }
                }

                for cid in to_park {
                    idle_pool.push(cid);
                    let idx = *carrier_idx_by_id.get(&cid).unwrap();
                    let pose = *parking_assign
                        .get(&cid)
                        .unwrap_or_else(|| parking_poses.first().expect("no parking poses computed"));
                    let cmds = cmds_by_carrier.get_mut(&cid).unwrap();
                    go_to_parking(inst, &mut carriers[idx], cmds, &mut reservation_table, pose);
                }
            }
        }

        if !any_left {
            break;
        }
    }

    // Produce plans in carrier order (and include all carriers)
    inst.carriers
        .iter()
        .map(|c| (c.id, cmds_by_carrier.remove(&c.id).unwrap_or_default()))
        .collect()
}
