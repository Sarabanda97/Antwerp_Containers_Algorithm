use std::collections::HashMap;
// CORREÇÃO: Removido 'ContainerLocation' do import abaixo porque ele é definido localmente neste ficheiro
use crate::model::{Id, Instance, Demand, Point, Rect, Direction}; 
use crate::planner::path::{go_to_pose, Command};
use crate::state::CarrierState;

// Definido localmente (não precisa de vir do model.rs)
#[derive(Clone, Debug)]
enum ContainerLocation {
    Storage  { storage_id: Id, depth: usize },
    Dispatch { dispatch_id: Id },
    OnCarrier { carrier_id: Id },
}

/// Estrutura auxiliar para não passar 10 argumentos em cada função.
/// Agrupa todo o estado mutável necessário para planear.
struct PlanningContext<'a> {
    inst: &'a Instance,
    c: &'a mut CarrierState,
    cmds: &'a mut Vec<Command>,
    storage_stacks: &'a mut Vec<Vec<Id>>,
    locs: &'a mut HashMap<Id, ContainerLocation>,
    dispatch_containers: &'a mut HashMap<Id, Vec<Id>>,
    
    // Índices rápidos
    storage_idx: &'a HashMap<Id, usize>,
    dispatch_idx: &'a HashMap<Id, usize>,
}

impl<'a> PlanningContext<'a> {
    
    /// Move o carrier para o staging point do alvo e alinha a direção.
    fn goto_staging(&mut self, target_bl: Point, target_dir: Direction) {
        go_to_pose(self.inst, self.c, target_bl, target_dir, self.cmds);
    }


    // --- Helpers: geometria/saídas do crane section ---------------------------------

    fn dims(dir: Direction) -> (i32, i32) {
        match dir {
            Direction::Up | Direction::Down => (4, 8),
            Direction::Left | Direction::Right => (8, 4),
        }
    }

    fn carrier_rect(&self) -> Rect {
        let (w,h) = Self::dims(self.c.dir);
        Rect { x1: self.c.bl.x, y1: self.c.bl.y, x2: self.c.bl.x + w - 1, y2: self.c.bl.y + h - 1 }
    }

    fn rects_intersect(a: &Rect, b: &Rect) -> bool {
        !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
    }

    fn crane_rect_for_dispatch(&self, dispatch_id: Id) -> Rect {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let crane_id = self.inst.dispatches[d_idx].crane_id;
        self.inst.cranes.iter().find(|c| c.id == crane_id).unwrap().rect
    }

    /// Garante que o carrier está FORA do crane section (para o crane poder pôr/tirar contentores do navio).
    fn ensure_outside_crane_section(&mut self, dispatch_id: Id) {
        let crane = self.crane_rect_for_dispatch(dispatch_id);
        while Self::rects_intersect(&self.carrier_rect(), &crane) {
            // Sair verticalmente (mais simples e sempre válido fora do yard)
            let target_y = crane.y2 + 1;
            self.goto_staging(Point { x: self.c.bl.x, y: target_y }, Direction::Up);
        }
    }

    /// Simula o aparecimento de um contentor na dispatch (navio -> dispatch).
    /// Regras simples: dispatch comporta 1 contentor (4×2), então limpamos o estado lógico antes.
    fn spawn_on_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        self.ensure_outside_crane_section(dispatch_id);

        if let Some(v) = self.dispatch_containers.get_mut(&dispatch_id) {
            v.clear();
            v.push(container_id);
        } else {
            self.dispatch_containers.insert(dispatch_id, vec![container_id]);
        }
        self.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });
    }

    /// Após descarregar para a dispatch (yard -> ship), o contentor sai do sistema.
    /// Para o checker aceitar a operação do crane, saímos imediatamente do crane section.
    fn finalize_delivery_to_ship(&mut self, dispatch_id: Id, container_id: Id) {
        self.ensure_outside_crane_section(dispatch_id);
        self.locs.remove(&container_id);
        if let Some(v) = self.dispatch_containers.get_mut(&dispatch_id) {
            v.clear();
        }
    }

    /// Executa LOAD a partir de uma Dispatch (Navio -> Carrier)
    fn load_from_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.unwrap();
        let dir = disp.staging_dir.unwrap();

        // 1. Ir para lá
        self.goto_staging(bl, dir);

        // 2. Verificar lógica
        let vec = self.dispatch_containers.get_mut(&dispatch_id).unwrap();
        let pos = vec.iter().position(|&x| x == container_id)
            .expect("Container não está na dispatch para load");
        vec.remove(pos);

        // 3. Executar comando
        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);

        // 4. Atualizar localização lógica
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
    }

    /// Executa UNLOAD para uma Dispatch (Carrier -> Navio)
    ///
    /// Importante: após deixar o contentor na dispatch, o crane vai levá-lo ao navio.
    /// Para isso, o carrier tem de sair do crane section imediatamente (regra do checker).
    fn unload_to_dispatch(&mut self, dispatch_id: Id, container_id: Id) {
        let d_idx = *self.dispatch_idx.get(&dispatch_id).unwrap();
        let disp = &self.inst.dispatches[d_idx];
        let bl = disp.staging_bl.unwrap();
        let dir = disp.staging_dir.unwrap();

        // 1) Ir ao staging da dispatch
        self.goto_staging(bl, dir);

        // 2) UNLOAD (deixa no chão)
        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;
        self.c.carrying = None;

        // 3) Atualizar estado lógico: o contentor fica na dispatch até o crane o recolher.
        if let Some(v) = self.dispatch_containers.get_mut(&dispatch_id) {
            v.push(container_id);
        } else {
            self.dispatch_containers.insert(dispatch_id, vec![container_id]);
        }
        self.locs.insert(container_id, ContainerLocation::Dispatch { dispatch_id });

        // 4) Garantir que o carrier sai do crane section (regra do checker)
        self.ensure_outside_crane_section(dispatch_id);
    }

    /// Executa LOAD a partir de um Storage (Yard -> Carrier)
 (Yard -> Carrier)
    /// NOTA: Assume que o contentor JÁ está no topo (ver ensure_container_accessible).
    fn load_from_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        // 1. Ir para lá
        self.goto_staging(bl, dir);

        // 2. Manipular stack
        let stack = &mut self.storage_stacks[s_idx];
        assert_eq!(stack.last(), Some(&container_id), "ERRO: Tentou carregar contentor que não está no topo!");
        stack.pop();

        // 3. Executar comando
        let t = self.c.time;
        self.cmds.push(Command::Load { t });
        self.c.time += 1;
        self.c.carrying = Some(container_id);

        // 4. Atualizar localização
        self.locs.insert(container_id, ContainerLocation::OnCarrier { carrier_id: self.c.id });
    }

    /// Executa UNLOAD para um Storage (Carrier -> Yard)
    fn unload_to_storage(&mut self, storage_id: Id, container_id: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        let stor = &self.inst.storages[s_idx];
        let bl = stor.staging_bl.unwrap();
        let dir = stor.staging_dir.unwrap();

        // 1. Ir para lá
        self.goto_staging(bl, dir);

        // 2. Verificar capacidade
        let stack = &mut self.storage_stacks[s_idx];
        if stack.len() >= 2 {
            panic!("ERRO: Tentativa de unload no storage {} que já está cheio (len=2)", storage_id);
        }
        
        // 3. Executar comando
        let t = self.c.time;
        self.cmds.push(Command::Unload { t });
        self.c.time += 1;
        self.c.carrying = None;

        // 4. Atualizar stack e localização
        stack.push(container_id);
        let depth = stack.len() - 1;
        self.locs.insert(container_id, ContainerLocation::Storage { storage_id, depth });
    }

    /// Encontra um storage temporário adequado (não cheio e preferencialmente vazio)
    fn find_temp_storage(&self, exclude_id: Id) -> Id {
        // Tenta achar um vazio
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id { continue; }
            if self.storage_stacks[i].is_empty() {
                return s.id;
            }
        }
        // Se não houver vazio, tenta um com espaço (len < 2)
        for (i, s) in self.inst.storages.iter().enumerate() {
            if s.id == exclude_id { continue; }
            if self.storage_stacks[i].len() < 2 {
                return s.id;
            }
        }
        panic!("ERRO CRÍTICO: Não há espaço no yard para reempilhar contentores!");
    }

    /// Garante que o contentor alvo está no topo da stack.
    /// Se estiver em baixo, move o de cima para outro lugar.
    fn ensure_container_accessible(&mut self, storage_id: Id, target_cid: Id) {
        let s_idx = *self.storage_idx.get(&storage_id).unwrap();
        
        // Verifica se precisamos de fazer algo
        let stack_len = self.storage_stacks[s_idx].len();
        if stack_len == 0 { panic!("Storage vazia, não contém {}", target_cid); }
        
        let top_cid = self.storage_stacks[s_idx][stack_len - 1];
        
        // Se o alvo já é o topo, estamos bem
        if top_cid == target_cid { return; }

        // Se não é o topo, é o de baixo. Precisamos mover o top_cid.
        // 1. Escolher destino temporário
        let temp_storage_id = self.find_temp_storage(storage_id);

        // 2. Mover top_cid -> temp_storage_id
        //    a) Load do topo
        self.load_from_storage(storage_id, top_cid);
        //    b) Unload no temp
        self.unload_to_storage(temp_storage_id, top_cid);
        
        // Agora target_cid deve estar no topo de storage_id
    }
}

// -----------------------------------------------------------------------------
// Função Principal
// -----------------------------------------------------------------------------

pub fn plan_all_demands(inst: &Instance) -> Vec<Command> {
    // 1. Preparar Estruturas de Dados
    let mut storage_stacks = inst.storage_stacks.clone();
    let mut dispatch_containers: HashMap<Id, Vec<Id>> = HashMap::new();
    let mut locs: HashMap<Id, ContainerLocation> = HashMap::new();
    
    // Inicializar locs e dispatch_containers
    for d in &inst.dispatches {
        dispatch_containers.insert(d.id, Vec::new());
    }
    for (s_idx, stack) in storage_stacks.iter().enumerate() {
        let sid = inst.storages[s_idx].id;
        for (depth, &cid) in stack.iter().enumerate() {
            locs.insert(cid, ContainerLocation::Storage { storage_id: sid, depth });
        }
    }

    // Índices rápidos
    let mut storage_idx = HashMap::new();
    for (i, s) in inst.storages.iter().enumerate() { storage_idx.insert(s.id, i); }
    let mut dispatch_idx = HashMap::new();
    for (i, d) in inst.dispatches.iter().enumerate() { dispatch_idx.insert(d.id, i); }

    // Carrier State Inicial (Assumindo 1 carrier por agora)
    let carrier_def = &inst.carriers[0];
    let mut c = CarrierState {
        id: carrier_def.id,
        bl: carrier_def.bl,
        dir: carrier_def.dir,
        carrying: None,
        time: 0,
    };
    let mut cmds = Vec::new();

    // 2. Criar o Contexto
    let mut ctx = PlanningContext {
        inst,
        c: &mut c,
        cmds: &mut cmds,
        storage_stacks: &mut storage_stacks,
        locs: &mut locs,
        dispatch_containers: &mut dispatch_containers,
        storage_idx: &storage_idx,
        dispatch_idx: &dispatch_idx,
    };

    // 3. Flatten Demands (navios + lista plana)
    // O parser separa ships e demands soltas, vamos juntar tudo sequencialmente para o toy
    let mut all_demands = Vec::new();
    
    // Prioridade aos navios
    for ship in &inst.ships {
        for op in &ship.operations {
            all_demands.push(op.clone());
        }
    }
    // Depois as soltas (se houver, no toy geralmente é uma ou outra)
    if inst.ships.is_empty() {
        for d in &inst.demands {
            all_demands.push(d.clone());
        }
    }

    // 4. Executar Demands
    for demand in all_demands {
        match demand {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // Navio -> Yard:
                // 1) garantir que o carrier está fora do crane section e "spawn" do contentor na dispatch (capacidade 1)
                ctx.spawn_on_dispatch(dispatch_id, container_id);

                // 2) ir à dispatch e carregar
                ctx.load_from_dispatch(dispatch_id, container_id);

                // 3) levar ao storage (empilha no topo; unload_to_storage verifica capacidade)
                ctx.unload_to_storage(storage_id, container_id);
            },

            Demand::Load { dispatch_id, container_id } => {
                // Cenário: Contentor está algures no Yard, precisa ir para Dispatch
                let current_loc = ctx.locs.get(&container_id).cloned().expect("Container perdido?");
                
                match current_loc {
                    ContainerLocation::Storage { storage_id, .. } => {
                        // Passo A: Resolver o problema da stack (se estiver por baixo)
                        ctx.ensure_container_accessible(storage_id, container_id);

                        // Passo B: Carregar do storage (agora garantido estar no topo)
                        ctx.load_from_storage(storage_id, container_id);

                        // Passo C: Levar à dispatch
                        ctx.unload_to_dispatch(dispatch_id, container_id);
                    },
                    _ => panic!("Load pede contentor que não está em Storage (está em {:?})", current_loc),
                }
            }
        }
    }

    cmds
}