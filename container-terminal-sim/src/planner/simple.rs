use std::collections::HashMap;

use crate::model::{Demand, Direction, Id, Instance, Point, Rect};
use crate::planner::path::{carrier_rect, go_to_pose, is_valid_pose, Command, ReservationTable};
use crate::state::CarrierState;

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

impl<'a> PlanningContext<'a> {
    fn goto_staging(&mut self, target_bl: Point, target_dir: Direction) {
        go_to_pose(self.inst, self.c, target_bl, target_dir, self.cmds, self.res);
    }

    fn reserve_idle(&mut self, dt: i32) {
        for _ in 0..dt {
            self.c.time += 1;
            self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
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

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
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

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
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

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
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

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
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
        reservation_table.reserve(0, carrier_rect(c.bl, c.dir));
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

    // Carriers per crane
    let mut carriers_per_crane: HashMap<Id, Vec<usize>> = HashMap::new();
    for (idx, c) in inst.carriers.iter().enumerate() {
        carriers_per_crane.entry(c.assigned_crane).or_default().push(idx);
    }

    // Crane time synchronization
    let mut crane_time: HashMap<Id, i32> = HashMap::new();
    for crane in &inst.cranes {
        crane_time.insert(crane.id, 0);
    }

    // Per-crane demand pointer
    let mut ptr: HashMap<Id, usize> = HashMap::new();
    for (crane_id, demands) in &demands_per_crane {
        ptr.insert(*crane_id, 0);
        if demands.is_empty() {
            crane_time.insert(*crane_id, 0);
        }
    }

    // Helper: score carrier for next demand
    let score_for = |carrier: &CarrierState, crane_id: Id, demand: &Demand, ready_t: i32| -> i32 {
        let t0 = carrier.time.max(ready_t);
        let (tx, ty) = match demand {
            Demand::Unload { dispatch_id, .. } => {
                let di = *dispatch_idx.get(dispatch_id).unwrap();
                let bl = inst.dispatches[di].staging_bl.unwrap();
                (bl.x, bl.y)
            }
            Demand::Load { dispatch_id, .. } => {
                let di = *dispatch_idx.get(dispatch_id).unwrap();
                let bl = inst.dispatches[di].staging_bl.unwrap();
                (bl.x, bl.y)
            }
        };
        let dist = (carrier.bl.x - tx).abs() + (carrier.bl.y - ty).abs();
        t0 + dist
    };

    // Main scheduling loop: process each crane's ordered demands, but choose carriers dynamically.
    loop {
        let mut any_left = false;

        for (crane_id, demands) in demands_per_crane.iter() {
            let i = *ptr.get(crane_id).unwrap_or(&0);
            if i >= demands.len() {
                continue;
            }
            any_left = true;

            let demand = demands[i].clone();
            let ready_t = *crane_time.get(crane_id).unwrap_or(&0);

            let carrier_indices = match carriers_per_crane.get(crane_id) {
                Some(v) if !v.is_empty() => v,
                _ => continue,
            };

            // Choose best carrier for this job
            let mut best_idx = carrier_indices[0];
            let mut best_score = i32::MAX;
            for &ci in carrier_indices {
                let sc = score_for(&carriers[ci], *crane_id, &demand, ready_t);
                if sc < best_score {
                    best_score = sc;
                    best_idx = ci;
                }
            }

            // Execute demand with chosen carrier
            {
                let c_id = carriers[best_idx].id;
                let cmds = cmds_by_carrier.get_mut(&c_id).unwrap();

                let mut ctx = PlanningContext {
                    inst,
                    c: &mut carriers[best_idx],
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
                    let dt = ready_t - ctx.c.time;
                    ctx.reserve_idle(dt);
                }

                match demand {
                    Demand::Unload { dispatch_id, container_id, storage_id } => {
                        // Ship unload only when carrier is OUTSIDE crane section
                        ensure_outside_crane_and_wait(inst, *crane_id, &mut ctx, 1);

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
                        ensure_outside_crane_and_wait(inst, *crane_id, &mut ctx, 1);

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
                crane_time.insert(*crane_id, ctx.c.time);
            }

            ptr.insert(*crane_id, i + 1);
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
