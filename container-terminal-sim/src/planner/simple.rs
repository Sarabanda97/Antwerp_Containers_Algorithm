use std::collections::HashMap;
use crate::model::{Id, Instance, Demand};
use crate::planner::path::{go_to_pose, Command};
use crate::state::CarrierState;

#[derive(Clone, Debug)]
enum ContainerLocation {
    Storage  { storage_id: Id, depth: usize },
    Dispatch { dispatch_id: Id },
    OnCarrier { carrier_id: Id },
}

type ContainerMap = HashMap<Id, ContainerLocation>;

fn build_initial_container_map(inst: &Instance) -> ContainerMap {
    let mut map = ContainerMap::new();

    for (s_idx, stack) in inst.storage_stacks.iter().enumerate() {
        let storage_id = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            map.insert(
                cid,
                ContainerLocation::Storage { storage_id, depth },
            );
        }
    }

    map
}

// índice rápido: storage_id -> índice no vetor inst.storages / storage_stacks
fn build_storage_index(inst: &Instance) -> HashMap<Id, usize> {
    let mut m = HashMap::new();
    for (idx, s) in inst.storages.iter().enumerate() {
        m.insert(s.id, idx);
    }
    m
}

/// índice rápido: dispatch_id -> índice no vetor inst.dispatches
fn build_dispatch_index(inst: &Instance) -> HashMap<Id, usize> {
    let mut m = HashMap::new();
    for (idx, d) in inst.dispatches.iter().enumerate() {
        m.insert(d.id, idx);
    }
    m
}

// Constroi container_location a partir de inst.storage_stacks inicial
fn init_container_locations(
    inst: &Instance,
    _storage_index: &HashMap<Id, usize>,
) -> ContainerMap {
    let mut locs: ContainerMap = HashMap::new();

    for (s_idx, stack) in inst.storage_stacks.iter().enumerate() {
        let storage_id = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            locs.insert(
                cid,
                ContainerLocation::Storage { storage_id, depth },
            );
        }
    }

    locs
}

// Helper: faz um load no tempo atual deste carrier e atualiza estado/lógica
fn do_load(
    c: &mut CarrierState,
    carrier_id: Id,
    container_id: Id,
    cmds: &mut Vec<Command>,
    locs: &mut ContainerMap,
) {
    let t = c.time;
    cmds.push(Command::Load { t });
    c.time += 1;

    c.carrying = Some(container_id);
    locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id });
}

// Helper: unload para storage (atualiza stack + localização)
fn do_unload_to_storage(
    c: &mut CarrierState,
    container_id: Id,
    storage_id: Id,
    storage_stacks: &mut Vec<Vec<Id>>,
    storage_index: &HashMap<Id, usize>,
    locs: &mut ContainerMap,
    cmds: &mut Vec<Command>,
) {
    let t = c.time;
    cmds.push(Command::Unload { t });
    c.time += 1;
    c.carrying = None;

    let s_idx = *storage_index
        .get(&storage_id)
        .expect("storage id inválido em do_unload_to_storage");

    let stack = &mut storage_stacks[s_idx];
    if stack.len() >= 2 {
        eprintln!(
            "[WARN] Storage {} já com 2 contentores, a empilhar mesmo assim.",
            storage_id
        );
    }
    stack.push(container_id);
    let depth = stack.len() - 1;

    locs.insert(
        container_id,
        ContainerLocation::Storage { storage_id, depth },
    );
}

// Helper: unload para dispatch (para o caso de LOAD d c → ship)
fn do_unload_to_dispatch(
    c: &mut CarrierState,
    container_id: Id,
    dispatch_id: Id,
    locs: &mut ContainerMap,
    cmds: &mut Vec<Command>,
) {
    let t = c.time;
    cmds.push(Command::Unload { t });
    c.time += 1;
    c.carrying = None;

    locs.insert(
        container_id,
        ContainerLocation::Dispatch { dispatch_id },
    );
}

pub fn plan_all_demands(inst: &Instance) -> Vec<Command> {
    // 1) estado local das stacks (copiado do instance)
    let mut storage_stacks: Vec<Vec<Id>> = inst.storage_stacks.clone();

    // 2) índices rápidos
    let storage_index = build_storage_index(inst);
    let dispatch_index = build_dispatch_index(inst);

    // 3) localização lógica dos contentores
    let mut locs = init_container_locations(inst, &storage_index);

    // 4) carrier único: construir CarrierState inicial a partir do instance
    let carrier_def = inst
        .carriers
        .get(0)
        .expect("Esperava pelo menos 1 carrier");

    let mut c = CarrierState {
        id:   carrier_def.id,
        bl:   carrier_def.bl,
        dir:  carrier_def.dir,
        carrying: None,
        time: 0,
    };

    let carrier_id = c.id;
    let mut cmds: Vec<Command> = Vec::new();

    // 5) percorre as demands em ordem
    for d in &inst.demands {
        match *d {
            Demand::Unload {
                dispatch_id,
                container_id,
                storage_id,
            } => {
                // container chega do navio → aparece na dispatch
                locs.insert(
                    container_id,
                    ContainerLocation::Dispatch { dispatch_id },
                );

                // 1) ir para staging da dispatch
                let d_idx = *dispatch_index
                    .get(&dispatch_id)
                    .expect("dispatch inexistente em UNLOAD");
                let disp = &inst.dispatches[d_idx];
                let dbl = disp
                    .staging_bl
                    .expect("dispatch sem staging_bl");
                let ddir = disp
                    .staging_dir
                    .expect("dispatch sem staging_dir");

                go_to_pose(inst, &mut c, dbl, ddir, &mut cmds);

                // 2) load do container da dispatch para o carrier
                do_load(
                    &mut c,
                    carrier_id,
                    container_id,
                    &mut cmds,
                    &mut locs,
                );

                // 3) ir para staging do storage s
                let s_idx = *storage_index
                    .get(&storage_id)
                    .expect("storage inexistente em UNLOAD");
                let stor = &inst.storages[s_idx];
                let sbl = stor
                    .staging_bl
                    .expect("storage sem staging_bl");
                let sdir = stor
                    .staging_dir
                    .expect("storage sem staging_dir");

                go_to_pose(inst, &mut c, sbl, sdir, &mut cmds);

                // 4) unload para storage (empilha no topo)
                do_unload_to_storage(
                    &mut c,
                    container_id,
                    storage_id,
                    &mut storage_stacks,
                    &storage_index,
                    &mut locs,
                    &mut cmds,
                );
            }

            Demand::Load {
                dispatch_id,
                container_id,
            } => {
                // container está no yard; descobrir onde
                let loc = locs
                    .get(&container_id)
                    .cloned()
                    .expect("Demand LOAD com container desconhecido");

                match loc {
                    ContainerLocation::Storage { storage_id, depth } => {
                        let s_idx = *storage_index
                            .get(&storage_id)
                            .expect("storage inexistente em LOAD");
                        let stack = &storage_stacks[s_idx];

                        if stack.last() != Some(&container_id) {
                            eprintln!(
                                "[WARN] LOAD do contentor {} que não está no topo da storage {} (depth {}). Ainda não tratamos reempilhamento.",
                                container_id, storage_id, depth
                            );
                        }

                        // 1) ir para staging desse storage
                        let stor = &inst.storages[s_idx];
                        let sbl = stor
                            .staging_bl
                            .expect("storage sem staging_bl");
                        let sdir = stor
                            .staging_dir
                            .expect("storage sem staging_dir");

                        go_to_pose(inst, &mut c, sbl, sdir, &mut cmds);

                        // 2) load do topo da stack
                        let stack_mut = &mut storage_stacks[s_idx];
                        let popped = stack_mut
                            .pop()
                            .expect("stack vazia ao fazer LOAD");
                        if popped != container_id {
                            eprintln!(
                                "[WARN] topo da storage {} era {}, não {}. A ajustar lógica no futuro.",
                                storage_id, popped, container_id
                            );
                        }

                        do_load(
                            &mut c,
                            carrier_id,
                            container_id,
                            &mut cmds,
                            &mut locs,
                        );

                        // 3) ir para staging da dispatch
                        let d_idx = *dispatch_index
                            .get(&dispatch_id)
                            .expect("dispatch inexistente em LOAD");
                        let disp = &inst.dispatches[d_idx];
                        let dbl = disp
                            .staging_bl
                            .expect("dispatch sem staging_bl");
                        let ddir = disp
                            .staging_dir
                            .expect("dispatch sem staging_dir");

                        go_to_pose(inst, &mut c, dbl, ddir, &mut cmds);

                        // 4) unload para dispatch (navio depois trata)
                        do_unload_to_dispatch(
                            &mut c,
                            container_id,
                            dispatch_id,
                            &mut locs,
                            &mut cmds,
                        );
                    }

                    ContainerLocation::Dispatch { dispatch_id: d2 } => {
                        eprintln!(
                            "[WARN] LOAD pediu contentor {} que já está na dispatch {}. Caso especial ainda não tratado.",
                            container_id, d2
                        );
                    }

                    ContainerLocation::OnCarrier { carrier_id: cid } => {
                        eprintln!(
                            "[WARN] LOAD do contentor {} mas já está em cima do carrier {}.",
                            container_id, cid
                        );
                    }
                }
            }
        }
    }

    cmds
}
