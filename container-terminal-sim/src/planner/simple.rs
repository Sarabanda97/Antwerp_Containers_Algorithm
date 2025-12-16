use std::collections::HashMap;

use crate::model::{Demand, Direction, Id, Instance, Point};
use crate::planner::path::{carrier_rect, go_to_pose, Command, ReservationTable};
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

impl<'a> PlanningContext<'a> {
    fn goto_staging(&mut self, target_bl: Point, target_dir: Direction) {
        go_to_pose(self.inst, self.c, target_bl, target_dir, self.cmds, self.res);
    }

    /// Slow but robust: scan yard stacks to find container if loc-map is missing/stale.
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

        // Ensure carrier is outside crane rect before ship operation
        if let Some(crane_id) = Some(disp.crane_id) {
            if let Some(cr) = self.inst.cranes.iter().find(|c| c.id == crane_id) {
                let r = crate::planner::path::carrier_rect(self.c.bl, self.c.dir);
                if crate::planner::path::rect_intersects(&r, &cr.rect) {
                    // choose a waiting pose just to the right of the crane
                    let wait_bl = Point { x: cr.rect.x2 + 1, y: self.c.bl.y };
                    self.goto_staging(wait_bl, Direction::Right);
                }
            }
        }

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
        self.locs
            .insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn unload_to_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).expect("dispatch_idx missing");
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.expect("Dispatch staging_bl missing");
        let dir = disp.staging_dir.expect("Dispatch staging_dir missing");

        // Ensure carrier is outside crane rect before ship operation
        if let Some(crane_id) = Some(disp.crane_id) {
            if let Some(cr) = self.inst.cranes.iter().find(|c| c.id == crane_id) {
                let r = crate::planner::path::carrier_rect(self.c.bl, self.c.dir);
                if crate::planner::path::rect_intersects(&r, &cr.rect) {
                    let wait_bl = Point { x: cr.rect.x2 + 1, y: self.c.bl.y };
                    self.goto_staging(wait_bl, Direction::Right);
                }
            }
        }

        self.goto_staging(bl, dir);

        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;

        self.c.carrying = None;

        self.dispatch_containers
            .entry(dispatch_id)
            .or_default()
            .push(container_id);
        self.locs
            .insert(container_id, ContainerLocation::Dispatch { dispatch_id });

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn load_from_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).expect("storage_idx missing");
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.expect("Storage staging_bl missing");
        let dir = stor.staging_dir.expect("Storage staging_dir missing");

        self.goto_staging(bl, dir);

        // Must be top (after ensure_container_accessible)
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
        self.locs
            .insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });

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

        self.locs
            .insert(container_id, ContainerLocation::Storage { storage_id, depth });

        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn find_best_temp_storage(&self, exclude_id: Id) -> Id {
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

        if let Some(id) = best_id {
            id
        } else {
            // Fallback: pick the storage with smallest stack (even if full)
            let mut min_len = usize::MAX;
            let mut pick: Option<Id> = None;
            for (i, s) in self.inst.storages.iter().enumerate() {
                if s.id == exclude_id { continue; }
                let l = self.storage_stacks[i].len();
                if l < min_len {
                    min_len = l;
                    pick = Some(s.id);
                }
            }
            pick.expect("FULL YARD: no storage available at all")
        }
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

            let temp_storage_id = self.find_best_temp_storage(storage_id);

            // Move the blocking container to temporary storage
            self.load_from_storage(storage_id, top_cid);
            self.unload_to_storage(temp_storage_id, top_cid);
        }
    }
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

    // Build ordered operations per crane
    let mut demands_per_crane: HashMap<Id, Vec<Demand>> = HashMap::new();
    for ship in &inst.ships {
        if let Some(crane) = ship.crane_id {
            let entry = demands_per_crane.entry(crane).or_default();
            for op in &ship.operations {
                entry.push(op.clone());
            }
        }
    }
    if inst.ships.is_empty() {
        let entry = demands_per_crane.entry(0).or_default();
        for d in &inst.demands {
            entry.push(d.clone());
        }
    }

    let mut carriers_per_crane: HashMap<Id, Vec<usize>> = HashMap::new();
    for (idx, c) in inst.carriers.iter().enumerate() {
        carriers_per_crane.entry(c.assigned_crane).or_default().push(idx);
    }

    // Strict order: all crane ops go to first carrier of that crane
    let mut tasks_per_carrier: HashMap<Id, Vec<Demand>> = HashMap::new();
    for (crane_id, demands) in demands_per_crane {
        if let Some(carrier_indices) = carriers_per_crane.get(&crane_id) {
            if carrier_indices.is_empty() {
                continue;
            }
            let carrier_idx = carrier_indices[0];
            let carrier_id = inst.carriers[carrier_idx].id;
            tasks_per_carrier.entry(carrier_id).or_default().extend(demands);
        }
    }

    let mut final_plans: Vec<(Id, Vec<Command>)> = Vec::new();

    for carrier_def in &inst.carriers {
        let mut c = CarrierState {
            id: carrier_def.id,
            bl: carrier_def.bl,
            dir: carrier_def.dir,
            carrying: None,
            time: 0,
        };

        let mut cmds: Vec<Command> = Vec::new();
        reservation_table.reserve(0, carrier_rect(c.bl, c.dir));

        let mut ctx = PlanningContext {
            inst,
            c: &mut c,
            cmds: &mut cmds,
            storage_stacks: &mut storage_stacks,
            locs: &mut locs,
            dispatch_containers: &mut dispatch_containers,
            storage_idx: &storage_idx,
            dispatch_idx: &dispatch_idx,
            res: &mut reservation_table,
        };

        let my_demands = tasks_per_carrier.remove(&carrier_def.id).unwrap_or_default();

        for demand in my_demands {
            match demand {
                Demand::Unload { dispatch_id, container_id, storage_id } => {
                    ctx.dispatch_containers.entry(dispatch_id).or_default().push(container_id);
                    ctx.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });

                    ctx.load_from_dispatch(dispatch_id, container_id);
                    ctx.unload_to_storage(storage_id, container_id);
                }
                Demand::Load { dispatch_id, container_id } => {
                    // robust location lookup
                    let loc = ctx
                        .locs
                        .get(&container_id)
                        .cloned()
                        .or_else(|| ctx.locate_and_register_in_stacks(container_id));

                    let loc = loc.unwrap_or_else(|| {
                        panic!("LOAD requested for container {} but its location is unknown (not in stacks/dispatch)", container_id)
                    });

                    match loc {
                        ContainerLocation::Storage { storage_id, .. } => {
                            ctx.ensure_container_accessible(storage_id, container_id);
                            ctx.load_from_storage(storage_id, container_id);
                            ctx.unload_to_dispatch(dispatch_id, container_id);
                        }
                        ContainerLocation::Dispatch { dispatch_id: d } => {
                            ctx.load_from_dispatch(d, container_id);
                            ctx.unload_to_dispatch(dispatch_id, container_id);
                        }
                        ContainerLocation::OnCarrier { .. } => {
                            panic!("Container {} already on some carrier while processing demand", container_id);
                        }
                    }
                }
            }
        }

        final_plans.push((carrier_def.id, cmds));
    }

    final_plans
}
