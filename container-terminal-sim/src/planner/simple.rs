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

// inicializa map dispatch_id -> stack de containers (vazio no início)
fn init_dispatch_containers(inst: &Instance) -> HashMap<Id, Vec<Id>> {
    let mut m = HashMap::new();
    for d in &inst.dispatches {
        m.insert(d.id, Vec::new());
    }
    m
}

// Helper: load a partir da dispatch (navio -> carrier)
fn do_load_from_dispatch(
    inst: &Instance,
    c: &mut CarrierState,
    carrier_id: Id,
    dispatch_id: Id,
    container_id: Id,
    dispatch_containers: &mut HashMap<Id, Vec<Id>>,
    dispatch_index: &HashMap<Id, usize>,
    locs: &mut ContainerMap,
    cmds: &mut Vec<Command>,
) {
    // pré-condições: carrier no staging da dispatch e vazio
    let d_idx = *dispatch_index
        .get(&dispatch_id)
        .expect("dispatch inexistente em do_load_from_dispatch");
    let disp = &inst.dispatches[d_idx];
    let expected_bl = disp.staging_bl.expect("dispatch sem staging_bl");
    let expected_dir = disp.staging_dir.expect("dispatch sem staging_dir");

    // reforçar posicionamento com go_to_pose para evitar pequenos desalinhamentos
    go_to_pose(inst, c, expected_bl, expected_dir, cmds);

    assert!(c.bl == expected_bl, "Carrier não está no staging da dispatch {}", dispatch_id);
    assert!(c.dir == expected_dir, "Carrier não está virado para staging da dispatch {}", dispatch_id);
    assert!(c.carrying.is_none(), "Carrier já está a transportar algo ao tentar LOAD from dispatch");
    
    // verificar que o container está na dispatch
    let vec = dispatch_containers
        .get_mut(&dispatch_id)
        .expect("dispatch missing in dispatch_containers");
    let pos = vec.iter().position(|&x| x == container_id);
    assert!(pos.is_some(), "container {} não está presente na dispatch {}", container_id, dispatch_id);
    vec.remove(pos.unwrap());

    let t = c.time;
    cmds.push(Command::Load { t });
    c.time += 1;

    c.carrying = Some(container_id);
    locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id });
}

// Helper: load a partir de storage (yard -> carrier)
fn do_load_from_storage(
    inst: &Instance,
    c: &mut CarrierState,
    carrier_id: Id,
    container_id: Id,
    storage_id: Id,
    storage_stacks: &mut Vec<Vec<Id>>,
    storage_index: &HashMap<Id, usize>,
    locs: &mut ContainerMap,
    cmds: &mut Vec<Command>,
) {
    let s_idx = *storage_index
        .get(&storage_id)
        .expect("storage inexistente em do_load_from_storage");
    let stor = &inst.storages[s_idx];
    let expected_bl = stor.staging_bl.expect("storage sem staging_bl");
    let expected_dir = stor.staging_dir.expect("storage sem staging_dir");

    // reforçar posicionamento com go_to_pose para evitar pequenos desalinhamentos
    go_to_pose(inst, c, expected_bl, expected_dir, cmds);
    // reforçar posicionamento com movimentos corretivos caso ainda haja desalinhamento
    if c.bl != expected_bl || c.dir != expected_dir {
        // tentativa corretiva adicional via go_to_pose
        go_to_pose(inst, c, expected_bl, expected_dir, cmds);
    }

    assert!(c.bl == expected_bl, "Carrier não está no staging da storage {}", storage_id);
    assert!(c.dir == expected_dir, "Carrier não está virado para staging da storage {}", storage_id);
    assert!(c.carrying.is_none(), "Carrier já está a transportar algo ao tentar LOAD from storage");

    let stack = &mut storage_stacks[s_idx];
    assert!(!stack.is_empty(), "storage {} vazia ao tentar LOAD", storage_id);
    // procurar o container na stack; se estiver no topo faz pop, senão remove na posição
    let pos_opt = stack.iter().position(|&x| x == container_id);
    if let Some(pos) = pos_opt {
        if pos == stack.len() - 1 {
            let _ = stack.pop().unwrap();
        } else {
            eprintln!(
                "[WARN] container {} não estava no topo da storage {} (pos {}). Removendo da stack.",
                container_id, storage_id, pos
            );
            stack.remove(pos);
        }
    } else {
        panic!("container {} não encontrado na storage {} ao tentar LOAD", container_id, storage_id);
    }

    let t = c.time;
    cmds.push(Command::Load { t });
    c.time += 1;

    c.carrying = Some(container_id);
    locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id });
}

// Helper: unload para storage (atualiza stack + localização)
fn do_unload_to_storage(
    inst: &Instance,
    c: &mut CarrierState,
    container_id: Id,
    storage_id: Id,
    storage_stacks: &mut Vec<Vec<Id>>,
    storage_index: &HashMap<Id, usize>,
    locs: &mut ContainerMap,
    cmds: &mut Vec<Command>,
) {
    // garantir posicionamento exato no staging antes de descarregar
    let s_idx = *storage_index
        .get(&storage_id)
        .expect("storage id inválido em do_unload_to_storage");
    let stor = &inst.storages[s_idx];
    let expected_bl = stor.staging_bl.expect("storage sem staging_bl");
    let expected_dir = stor.staging_dir.expect("storage sem staging_dir");
    go_to_pose(inst, c, expected_bl, expected_dir, cmds);

    // pré-condição: carrier carrega o container
    assert!(c.carrying.is_some(), "Carrier não carrega nada ao tentar UNLOAD para storage {}", storage_id);

    let t = c.time;
    cmds.push(Command::Unload { t });
    c.time += 1;
    c.carrying = None;

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
    assert!(c.carrying.is_some(), "Carrier não carrega nada ao tentar UNLOAD para dispatch {}", dispatch_id);
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
    // 3b) estado lógico dos containers nas dispatches (vazio inicialmente)
    let mut dispatch_containers: HashMap<Id, Vec<Id>> = init_dispatch_containers(inst);

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

    // 5) percorre as ships em ordem e as suas operations (mantém ordem por ship)
    if !inst.ships.is_empty() {
        for ship in &inst.ships {
            for d in &ship.operations {
                match *d {
                    Demand::Unload { dispatch_id, container_id, storage_id } => {
                        // container chega do navio → aparece na dispatch
                        locs.insert(
                            container_id,
                            ContainerLocation::Dispatch { dispatch_id },
                        );
                        // também regista nos containers da dispatch
                        if let Some(vec) = dispatch_containers.get_mut(&dispatch_id) {
                            vec.push(container_id);
                        } else {
                            dispatch_containers.insert(dispatch_id, vec![container_id]);
                        }

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
                        do_load_from_dispatch(
                            inst,
                            &mut c,
                            carrier_id,
                            dispatch_id,
                            container_id,
                            &mut dispatch_containers,
                            &dispatch_index,
                            &mut locs,
                            &mut cmds,
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
                            inst,
                            &mut c,
                            container_id,
                            storage_id,
                            &mut storage_stacks,
                            &storage_index,
                            &mut locs,
                            &mut cmds,
                        );
                    }

                    Demand::Load { dispatch_id, container_id } => {
                        // encontrar onde está o container
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
                                do_load_from_storage(
                                    inst,
                                    &mut c,
                                    carrier_id,
                                    container_id,
                                    storage_id,
                                    &mut storage_stacks,
                                    &storage_index,
                                    &mut locs,
                                    &mut cmds,
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
                                if let Some(vec) = dispatch_containers.get_mut(&dispatch_id) {
                                    vec.push(container_id);
                                } else {
                                    dispatch_containers.insert(dispatch_id, vec![container_id]);
                                }
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
        }
    } else {
        // fallback: process flat demands vector if ships not present
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
                    // também regista nos containers da dispatch
                    if let Some(vec) = dispatch_containers.get_mut(&dispatch_id) {
                        vec.push(container_id);
                    } else {
                        dispatch_containers.insert(dispatch_id, vec![container_id]);
                    }

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
                    do_load_from_dispatch(
                        inst,
                        &mut c,
                        carrier_id,
                        dispatch_id,
                        container_id,
                        &mut dispatch_containers,
                        &dispatch_index,
                        &mut locs,
                        &mut cmds,
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
                        inst,
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

                            // 2) load do topo da stack (do_load_from_storage fará o pop e as verificações)
                            do_load_from_storage(
                                inst,
                                &mut c,
                                carrier_id,
                                container_id,
                                storage_id,
                                &mut storage_stacks,
                                &storage_index,
                                &mut locs,
                                &mut cmds,
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
    }

    cmds
}
