use std::collections::HashMap;
use crate::model::{Id, Instance, Demand, Point, Direction};
use crate::planner::path::{go_to_pose, Command, ReservationTable, carrier_rect};
use crate::state::CarrierState;

#[derive(Clone, Debug)]
enum ContainerLocation {
    Storage  { storage_id: Id, depth: usize },
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

    fn load_from_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.expect("Staging nulo");
        let dir = disp.staging_dir.expect("Dir nulo");

        self.goto_staging(bl, dir);

        let vec = self.dispatch_containers.get_mut(&dispatch_id).unwrap();
        if let Some(pos) = vec.iter().position(|&x| x == container_id) {
            vec.remove(pos);
        }

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
        
        // FIX: Reservar o tempo PÓS-ação para garantir que ninguém entra no nosso espaço enquanto carregamos
        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn unload_to_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.unwrap();
        let dir = disp.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;
        self.c.carrying = None;

        self.dispatch_containers.entry(dispatch_id).or_default().push(container_id);
        self.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });
        
        // FIX
        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn load_from_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        let stack = &mut self.storage_stacks[s_idx];
        stack.pop();

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
        
        // FIX
        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn unload_to_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        let stack = &mut self.storage_stacks[s_idx];
        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;
        self.c.carrying = None;

        stack.push(container_id);
        let depth = stack.len() - 1;
        self.locs.insert(container_id, ContainerLocation::Storage { storage_id, depth });
        
        // FIX
        self.res.reserve(self.c.time, carrier_rect(self.c.bl, self.c.dir));
    }

    fn find_best_temp_storage(&self, exclude_id: Id) -> Id {
        let current_pos = self.c.bl;
        let mut best_id = None;
        let mut min_dist = i32::MAX;

        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id { continue; }
            if self.storage_stacks[i].len() < 2 {
                if let Some(target_bl) = s.staging_bl {
                    let dist = (target_bl.x - current_pos.x).abs() + (target_bl.y - current_pos.y).abs();
                    let penalty = if self.storage_stacks[i].len() > 0 { 200 } else { 0 };
                    let score = dist + penalty;
                    if score < min_dist {
                        min_dist = score;
                        best_id = Some(s.id);
                    }
                }
            }
        }
        best_id.expect("FULL YARD")
    }

    fn ensure_container_accessible(&mut self, storage_id: Id, target_cid: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stack_len = self.storage_stacks[s_idx].len();
        if stack_len == 0 { return; }
        let top_cid = self.storage_stacks[s_idx][stack_len - 1];
        if top_cid == target_cid { return; }

        let temp_storage_id = self.find_best_temp_storage(storage_id);
        self.load_from_storage(storage_id, top_cid);
        self.unload_to_storage(temp_storage_id, top_cid);
    }
}

pub fn plan_all_demands_multi(inst: &Instance) -> Vec<(Id, Vec<Command>)> {
    let mut storage_stacks = inst.storage_stacks.clone();
    let mut dispatch_containers: HashMap<Id, Vec<Id>> = HashMap::new();
    let mut locs: HashMap<Id, ContainerLocation> = HashMap::new();
    let mut reservation_table = ReservationTable::new();

    for d in &inst.dispatches { dispatch_containers.insert(d.id, Vec::new()); }
    for (s_idx, stack) in storage_stacks.iter().enumerate() {
        let sid = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            locs.insert(cid, ContainerLocation::Storage { storage_id: sid, depth });
        }
    }

    let mut storage_idx = HashMap::new();
    for (i, s) in inst.storages.iter().enumerate() { storage_idx.insert(s.id, i); }
    let mut dispatch_idx = HashMap::new();
    for (i, d) in inst.dispatches.iter().enumerate() { dispatch_idx.insert(d.id, i); }

    let mut demands_per_crane: HashMap<Id, Vec<Demand>> = HashMap::new();
    for ship in &inst.ships {
        if let Some(crane) = ship.crane_id {
            let entry = demands_per_crane.entry(crane).or_default();
            for op in &ship.operations { entry.push(op.clone()); }
        }
    }
    if inst.ships.is_empty() {
        let entry = demands_per_crane.entry(0).or_default();
        for d in &inst.demands { entry.push(d.clone()); }
    }

    let mut carriers_per_crane: HashMap<Id, Vec<usize>> = HashMap::new();
    for (idx, c) in inst.carriers.iter().enumerate() {
        carriers_per_crane.entry(c.assigned_crane).or_default().push(idx);
    }

    let mut tasks_per_carrier: HashMap<Id, Vec<Demand>> = HashMap::new();
    for (crane_id, demands) in demands_per_crane {
        if let Some(carrier_indices) = carriers_per_crane.get(&crane_id) {
            let num_carriers = carrier_indices.len();
            for (i, demand) in demands.iter().enumerate() {
                let carrier_idx = carrier_indices[i % num_carriers];
                let carrier_id = inst.carriers[carrier_idx].id;
                tasks_per_carrier.entry(carrier_id).or_default().push(demand.clone());
            }
        }
    }

    let mut final_plans = Vec::new();

    for carrier_def in &inst.carriers {
        let mut c = CarrierState {
            id: carrier_def.id,
            bl: carrier_def.bl,
            dir: carrier_def.dir,
            carrying: None,
            time: 0, 
        };
        let mut cmds = Vec::new();
        
        reservation_table.reserve(0, carrier_rect(c.bl, c.dir));

        let mut ctx = PlanningContext {
            inst, c: &mut c, cmds: &mut cmds, storage_stacks: &mut storage_stacks,
            locs: &mut locs, dispatch_containers: &mut dispatch_containers,
            storage_idx: &storage_idx, dispatch_idx: &dispatch_idx,
            res: &mut reservation_table,
        };

        let my_demands = tasks_per_carrier.remove(&carrier_def.id).unwrap_or_default();

        for demand in my_demands {
            match demand {
                Demand::Unload { dispatch_id, container_id, storage_id } => {
                    if let Some(v) = ctx.dispatch_containers.get_mut(&dispatch_id) { v.push(container_id); }
                    ctx.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });
                    ctx.load_from_dispatch(dispatch_id, container_id);
                    ctx.unload_to_storage(storage_id, container_id);
                },
                Demand::Load { dispatch_id, container_id } => {
                    if let Some(current_loc) = ctx.locs.get(&container_id).cloned() {
                        if let ContainerLocation::Storage { storage_id, .. } = current_loc {
                            ctx.ensure_container_accessible(storage_id, container_id);
                            ctx.load_from_storage(storage_id, container_id);
                            ctx.unload_to_dispatch(dispatch_id, container_id);
                        }
                    }
                }
            }
        }
        final_plans.push((carrier_def.id, cmds));
    }
    final_plans
}