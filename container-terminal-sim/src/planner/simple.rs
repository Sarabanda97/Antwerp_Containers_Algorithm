use std::collections::{HashMap, HashSet};

use crate::model::{Demand, Direction, Id, Instance, Point, Rect};
use crate::planner::path::{go_to_pose, Command};
use crate::state::CarrierState;

#[derive(Clone, Debug)]
enum ContainerLocation {
    Storage { storage_id: Id, depth: usize },
    Dispatch { dispatch_id: Id },
    OnCarrier { carrier_id: Id },
}

// Debug: IDs to trace closely (adjust as needed)
const DEBUG_CIDS: &[Id] = &[6, 3];

struct PlanningContext<'a> {
    inst: &'a Instance,
    c: &'a mut CarrierState,
    cmds: &'a mut Vec<Command>,

    storage_stacks: &'a mut Vec<Vec<Id>>,
    locs: &'a mut HashMap<Id, ContainerLocation>,
    dispatch_containers: &'a mut HashMap<Id, Vec<Id>>,

    storage_idx: &'a HashMap<Id, usize>,
    dispatch_idx: &'a HashMap<Id, usize>,

    // Containers placed by ship unloads (prefer not to disturb, but we allow temporarily if needed)
    protected: &'a mut HashSet<Id>,

    // cid -> “home storage” (where it should end up)
    final_dest: &'a mut HashMap<Id, Id>,
}

impl<'a> PlanningContext<'a> {
    fn is_debug_cid(&self, cid: Id) -> bool {
        DEBUG_CIDS.iter().any(|&x| x == cid)
    }

    fn goto_staging(&mut self, target_bl: Point, target_dir: Direction) {
        go_to_pose(self.inst, self.c, target_bl, target_dir, self.cmds);
    }

    // ----------------- crane section safety -----------------

    fn dims(dir: Direction) -> (i32, i32) {
        match dir {
            Direction::Up | Direction::Down => (4, 8),
            Direction::Left | Direction::Right => (8, 4),
        }
    }

    fn carrier_rect(&self) -> Rect {
        let (w, h) = Self::dims(self.c.dir);
        Rect {
            x1: self.c.bl.x,
            y1: self.c.bl.y,
            x2: self.c.bl.x + w - 1,
            y2: self.c.bl.y + h - 1,
        }
    }

    fn rects_intersect(a: &Rect, b: &Rect) -> bool {
        !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
    }

    fn crane_rect_for_dispatch(&self, dispatch_id: Id) -> Rect {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let crane_id = self.inst.dispatches[d_idx].crane_id;
        self.inst.cranes.iter().find(|c| c.id == crane_id).unwrap().rect
    }

    fn ensure_outside_crane_section(&mut self, dispatch_id: Id) {
        let crane = self.crane_rect_for_dispatch(dispatch_id);
        while Self::rects_intersect(&self.carrier_rect(), &crane) {
            let target_y = crane.y2 + 1;
            self.goto_staging(Point { x: self.c.bl.x, y: target_y }, Direction::Up);
        }
    }

    fn spawn_on_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        self.ensure_outside_crane_section(dispatch_id);

        self.dispatch_containers.entry(dispatch_id).or_default().push(container_id);


        self.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });

        if self.is_debug_cid(container_id) {
            eprintln!(
                "[DBG spawn_on_dispatch] dispatch={} container={} vec={:?} loc={:?}",
                dispatch_id,
                container_id,
                self.dispatch_containers.get(&dispatch_id),
                self.locs.get(&container_id)
            );
        }
    }

    fn finalize_delivery_to_ship(&mut self, dispatch_id: Id, container_id: Id) {
        self.ensure_outside_crane_section(dispatch_id);

        self.locs.remove(&container_id);
        if let Some(v) = self.dispatch_containers.get_mut(&dispatch_id) {
            v.clear();
        }

        if self.is_debug_cid(container_id) {
            eprintln!(
                "[DBG finalize_delivery_to_ship] dispatch={} container={} loc_present={}",
                dispatch_id,
                container_id,
                self.locs.contains_key(&container_id)
            );
        }
    }

    // ----------------- dispatch load/unload -----------------

    fn load_from_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.unwrap();
        let dir = disp.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        // logical remove from dispatch vector
        let vec = self.dispatch_containers.get_mut(&dispatch_id).unwrap();
        let pos = vec.iter().position(|&x| x == container_id)
            .expect("Container não está na dispatch para load");
        vec.remove(pos);

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);

        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
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


        self.ensure_outside_crane_section(dispatch_id);
    }

    // ----------------- storage load/unload -----------------

    fn load_from_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        let stack = &mut self.storage_stacks[s_idx];
        assert_eq!(
            stack.last(),
            Some(&container_id),
            "ERRO: Tentou carregar contentor que não está no topo!"
        );
        stack.pop();

        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);

        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
    }

    fn unload_to_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        self.goto_staging(bl, dir);

        let stack = &mut self.storage_stacks[s_idx];
        if stack.len() >= 2 {
            panic!(
                "ERRO: Tentativa de unload no storage {} que já está cheio (len=2)",
                storage_id
            );
        }

        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;
        self.c.carrying = None;

        stack.push(container_id);
        let depth = stack.len() - 1;
        self.locs.insert(container_id, ContainerLocation::Storage { storage_id, depth });
    }

    // ----------------- stack helpers -----------------

    fn find_buffer_storage(&self, exclude: Id) -> Option<Id> {
        // prefer empty
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude { continue; }
            if self.storage_stacks[i].is_empty() { return Some(s.id); }
        }
        // then any with space
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude { continue; }
            if self.storage_stacks[i].len() < 2 { return Some(s.id); }
        }
        None
    }

    fn find_temp_storage_excluding(&self, exclude_id: Id, forbidden: &HashSet<Id>) -> Option<Id> {
        // empty first
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id { continue; }
            if forbidden.contains(&s.id) { continue; }
            if self.storage_stacks[i].is_empty() {
                return Some(s.id);
            }
        }
        // then any with space
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id { continue; }
            if forbidden.contains(&s.id) { continue; }
            if self.storage_stacks[i].len() < 2 {
                return Some(s.id);
            }
        }
        None
    }

    /// Ensure target is accessible: if it's in bottom, move top away to a buffer.
    /// If the top is protected, we still allow moving it, but we register its “home” to put it back later.
    fn ensure_container_accessible(&mut self, storage_id: Id, target_cid: Id) -> Result<(), ()> {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();

        if self.storage_stacks[s_idx].is_empty() {
            return Err(());
        }

        let stack_len = self.storage_stacks[s_idx].len();
        let top_cid = self.storage_stacks[s_idx][stack_len - 1];

        if top_cid == target_cid {
            return Ok(());
        }

        // target must be bottom (height max 2)
        if self.protected.contains(&top_cid) {
            self.final_dest.entry(top_cid).or_insert(storage_id);
        }

        let forbidden: HashSet<Id> = HashSet::new();
        let temp = self
            .find_temp_storage_excluding(storage_id, &forbidden)
            .or_else(|| self.find_buffer_storage(storage_id))
            .ok_or(())?;

        self.load_from_storage(storage_id, top_cid);
        self.unload_to_storage(temp, top_cid);

        Ok(())
    }

    /// For tiny instances: just use preferred if it has space; otherwise any buffer with space.
    fn choose_storage_for_ship_unload(&self, preferred: Id) -> Id {
        let s_idx = *self.storage_idx.get(&preferred).unwrap();
        if self.storage_stacks[s_idx].len() < 2 {
            return preferred;
        }
        self.find_buffer_storage(preferred)
            .unwrap_or_else(|| panic!("ERRO CRÍTICO: yard cheio, sem espaço para unload!"))
    }

    /// Try to move deviated containers back “home” if possible (optional, helps avoid messy yard).
    fn attempt_relocate_finalized(&mut self) {
        let entries: Vec<(Id, Id)> = self.final_dest.iter().map(|(&k, &v)| (k, v)).collect();

        for (cid, preferred) in entries {
            let cur_loc = match self.locs.get(&cid).cloned() {
                Some(l) => l,
                None => continue,
            };

            match cur_loc {
                ContainerLocation::Storage { storage_id, .. } => {
                    if storage_id == preferred {
                        self.final_dest.remove(&cid);
                        self.protected.insert(cid);
                        continue;
                    }

                    if self.ensure_container_accessible(storage_id, cid).is_err() {
                        continue;
                    }

                    self.load_from_storage(storage_id, cid);
                    self.unload_to_storage(preferred, cid);

                    self.final_dest.remove(&cid);
                    self.protected.insert(cid);
                }
                ContainerLocation::OnCarrier { .. } => {
                    self.unload_to_storage(preferred, cid);
                    self.final_dest.remove(&cid);
                    self.protected.insert(cid);
                }
                ContainerLocation::Dispatch { .. } => {
                    // should not happen here
                    continue;
                }
            }
        }
    }
}

// -----------------------------------------------------------------------------
// MAIN
// -----------------------------------------------------------------------------

pub fn plan_all_demands(inst: &Instance) -> Vec<Command> {
    let mut storage_stacks = inst.storage_stacks.clone();
    let mut dispatch_containers: HashMap<Id, Vec<Id>> = HashMap::new();
    let mut locs: HashMap<Id, ContainerLocation> = HashMap::new();
    let mut protected: HashSet<Id> = HashSet::new();
    let mut final_dest: HashMap<Id, Id> = HashMap::new();

    for d in &inst.dispatches {
        dispatch_containers.insert(d.id, Vec::new());
    }
    for (s_idx, stack) in storage_stacks.iter().enumerate() {
        let sid = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            locs.insert(cid, ContainerLocation::Storage { storage_id: sid, depth });
        }
    }

    let mut storage_idx = HashMap::new();
    for (i, s) in inst.storages.iter().enumerate() {
        storage_idx.insert(s.id, i);
    }
    let mut dispatch_idx = HashMap::new();
    for (i, d) in inst.dispatches.iter().enumerate() {
        dispatch_idx.insert(d.id, i);
    }

    // 1 carrier
    let carrier_def = &inst.carriers[0];
    let mut c = CarrierState {
        id: carrier_def.id,
        bl: carrier_def.bl,
        dir: carrier_def.dir,
        carrying: None,
        time: 0,
    };

    let mut cmds: Vec<Command> = Vec::new();

    let mut ctx = PlanningContext {
        inst,
        c: &mut c,
        cmds: &mut cmds,
        storage_stacks: &mut storage_stacks,
        locs: &mut locs,
        dispatch_containers: &mut dispatch_containers,
        storage_idx: &storage_idx,
        dispatch_idx: &dispatch_idx,
        protected: &mut protected,
        final_dest: &mut final_dest,
    };

    // Flatten demands: ships first (tiny instances use ships)
    let mut pending: Vec<Demand> = Vec::new();
    for ship in &inst.ships {
        for op in &ship.operations {
            pending.push(op.clone());
        }
    }
    if pending.is_empty() {
        pending.extend(inst.demands.iter().cloned());
    }

    while let Some(demand) = pending.first().cloned() {
        pending.remove(0);

        match demand {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // ship -> dispatch (spawn), then carrier picks and places in storage
                ctx.spawn_on_dispatch(dispatch_id, container_id);
                ctx.load_from_dispatch(dispatch_id, container_id);

                let chosen = ctx.choose_storage_for_ship_unload(storage_id);

                if chosen != storage_id {
                    ctx.final_dest.insert(container_id, storage_id);
                }

                ctx.unload_to_storage(chosen, container_id);

                // mark as protected if it ended in requested place; otherwise it will be protected when relocated
                if chosen == storage_id {
                    ctx.protected.insert(container_id);
                }

                // optional cleanup
                ctx.attempt_relocate_finalized();
            }

            Demand::Load { dispatch_id, container_id } => {
                // yard -> dispatch -> ship
                let current_loc = ctx.locs.get(&container_id).cloned().expect("Container perdido?");

                match current_loc {
                    ContainerLocation::Storage { storage_id, .. } => {
                        // ensure it is on top
                        if ctx.ensure_container_accessible(storage_id, container_id).is_err() {
                            // if we can't access now, push to end and continue
                            pending.push(Demand::Load { dispatch_id, container_id });
                            continue;
                        }

                        ctx.load_from_storage(storage_id, container_id);
                        ctx.unload_to_dispatch(dispatch_id, container_id);
                        ctx.finalize_delivery_to_ship(dispatch_id, container_id);

                        // optional cleanup
                        ctx.attempt_relocate_finalized();
                    }
                    _ => panic!("Load pede contentor que não está em Storage: {:?}", current_loc),
                }
            }
        }
    }

    cmds
}
