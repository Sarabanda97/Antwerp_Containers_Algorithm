use crate::model::*;
use crate::state::*;
use crate::planner::path::{Command, go_to_pose};

pub fn plan_all_demands(inst: &Instance) -> Vec<Command> {
    let mut world = WorldState::from_instance(inst);
    let carrier_id = inst.carriers[0].id;
    let c = world.carriers.iter_mut().find(|c| c.id == carrier_id).unwrap();

    let mut cmds = Vec::new();

    for d in &inst.demands {
        match *d {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // 1) ir ao staging da dispatch
                let disp = inst.dispatches.iter().find(|dd| dd.id == dispatch_id).unwrap();
                let db = disp.staging_bl.unwrap();
                let dd = disp.staging_dir.unwrap();
                go_to_pose(inst, c, db, dd, &mut cmds);

                // 2) load
                let t = c.time;
                cmds.push(Command::Load { t });
                c.time += 1;
                c.carrying = Some(container_id);

                // 3) ir ao staging do storage
                let stor_idx = inst.storages.iter().position(|s| s.id == storage_id).unwrap();
                let stor = &inst.storages[stor_idx];
                let sb = stor.staging_bl.unwrap();
                let sd = stor.staging_dir.unwrap();
                go_to_pose(inst, c, sb, sd, &mut cmds);

                // 4) unload
                let t = c.time;
                cmds.push(Command::Unload { t });
                c.time += 1;
                c.carrying = None;
                world.storage_stacks[stor_idx].push(container_id);
            }

            Demand::Load { dispatch_id, container_id } => {
                // 1) descobrir storage
                let (s_idx, _) = world.storage_stacks.iter().enumerate()
                    .find(|(_, st)| st.contains(&container_id))
                    .expect("container not found in any storage");

                // 2) staging do storage
                let stor = &inst.storages[s_idx];
                let sb = stor.staging_bl.unwrap();
                let sd = stor.staging_dir.unwrap();
                go_to_pose(inst, c, sb, sd, &mut cmds);

                // 3) load
                let t = c.time;
                cmds.push(Command::Load { t });
                c.time += 1;
                c.carrying = Some(container_id);
                // tirar do topo
                let st = &mut world.storage_stacks[s_idx];
                let pos = st.iter().position(|x| *x == container_id).unwrap();
                st.remove(pos);

                // 4) staging da dispatch
                let disp = inst.dispatches.iter().find(|dd| dd.id == dispatch_id).unwrap();
                let db = disp.staging_bl.unwrap();
                let dd = disp.staging_dir.unwrap();
                go_to_pose(inst, c, db, dd, &mut cmds);

                // 5) unload
                let t = c.time;
                cmds.push(Command::Unload { t });
                c.time += 1;
                c.carrying = None;
            }
        }
    }

    cmds
}
