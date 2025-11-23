use crate::model::*;
use anyhow::{anyhow, Result};
use std::collections::HashMap;

/* ==========================================================================
   ESTRUTURAS DE ESTADO
   ========================================================================== */

struct WorldState {
    container_locs: HashMap<Id, Loc>,
    storage_stacks: Vec<Vec<Id>>,
    car: CarState,
    out: Vec<String>,
}

#[derive(Debug, Clone)]
struct CarState {
    id: Id,
    pos: Point,
    dir: Direction,
    carrying: Option<Id>,
    t: i32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Loc {
    InStorage(usize),
    OnShip,
    OnCarrier,
}

/* ==========================================================================
   CONSTANTES & AUXILIARES
   ========================================================================== */

/// Zona segura para movimentos laterais. 
/// Deve ser menor que a base da Crane Section (Y=44) e Storages (Y=59).
/// Y=30 dá margem segura para o Carrier rodar (4x8 ou 8x4) sem tocar em nada.
const HIGHWAY_Y: i32 = 30;

fn dir_str(d: Direction) -> &'static str {
    match d { Direction::Up => "up", Direction::Down => "down", Direction::Left => "left", Direction::Right => "right" }
}

/* ==========================================================================
   LÓGICA DE MOVIMENTO "EM U" (ROBÓTICA)
   ========================================================================== */

/// Move apenas no eixo Y.
fn move_y_only(world: &mut WorldState, target_y: i32) {
    let dy = target_y - world.car.pos.y;
    if dy == 0 { return; }

    // Garante orientação vertical para andar em Y (Corredores)
    let needed_dir = if dy > 0 { Direction::Up } else { Direction::Down };
    
    // Se não estiver alinhado, roda.
    if world.car.dir != Direction::Up && world.car.dir != Direction::Down {
        face_dir(world, needed_dir);
    } else if world.car.dir != needed_dir {
        // Opcional: Rodar para a frente do movimento (estética e segurança)
        face_dir(world, needed_dir);
    }

    world.out.push(format!("{} move {}", world.car.t, dy.abs()));
    world.car.t += dy.abs();
    world.car.pos.y = target_y;
}

/// Move apenas no eixo X (Apenas seguro na Highway).
fn move_x_only(world: &mut WorldState, target_x: i32) {
    let dx = target_x - world.car.pos.x;
    if dx == 0 { return; }

    let needed_dir = if dx > 0 { Direction::Right } else { Direction::Left };
    
    if world.car.dir != needed_dir {
        face_dir(world, needed_dir);
    }

    world.out.push(format!("{} move {}", world.car.t, dx.abs()));
    world.car.t += dx.abs();
    world.car.pos.x = target_x;
}

/// Helper para rodar e atualizar a geometria do Carrier (Pivô).
fn face_dir(world: &mut WorldState, new_dir: Direction) {
    if world.car.dir == new_dir { return; }
    
    world.out.push(format!("{} face {}", world.car.t, dir_str(new_dir)));
    world.car.t += 1;
    
    // Atualização precisa da posição do Bottom-Left após rotação
    let old = world.car.dir;
    let (dx, dy) = match (old, new_dir) {
        (Direction::Down, Direction::Right) => (2, -2), (Direction::Right, Direction::Up) => (2, 2),
        (Direction::Up, Direction::Left) => (-2, 2), (Direction::Left, Direction::Down) => (-2, -2),
        (Direction::Down, Direction::Left) => (-2, -2), (Direction::Left, Direction::Up) => (-2, 2),
        (Direction::Up, Direction::Right) => (2, 2), (Direction::Right, Direction::Down) => (2, -2),
        _ => (0, 0), // Rotações de 180 (Up<->Down) não mudam BL logicamente neste modelo simplificado
    };
    
    world.car.dir = new_dir;
    world.car.pos.x += dx;
    world.car.pos.y += dy;
}

/* ==========================================================================
   NAVEGAÇÃO SEGURA
   ========================================================================== */

/// Algoritmo "Sair -> Alinhar -> Entrar"
fn drive_to_exact(world: &mut WorldState, target_x: i32, target_y: i32) -> Result<()> {
    // 1. SAIR: Se não estamos na coluna certa, temos de descer à estrada.
    if world.car.pos.x != target_x {
        move_y_only(world, HIGHWAY_Y);   // Desce para Y=30
        move_x_only(world, target_x);    // Desliza para X=24 ou X=6...
    }

    // 2. ENTRAR: Sobe/Desce até ao alvo exato.
    move_y_only(world, target_y);
    
    // 3. Opcional: Orientação final. Para interagir, Up ou Down costuma ser aceite.
    // O move_y_only já nos deixa virados para o alvo (Up se viemos de baixo).
    Ok(())
}

/* ==========================================================================
   AÇÕES
   ========================================================================== */

fn action_load(world: &mut WorldState, cid: Id) -> Result<()> {
    world.out.push(format!("{} load", world.car.t));
    world.car.t += 1;
    world.car.carrying = Some(cid);
    world.container_locs.insert(cid, Loc::OnCarrier);
    Ok(())
}

fn action_unload(world: &mut WorldState) -> Result<Id> {
    let cid = world.car.carrying.ok_or_else(|| anyhow!("Carrier vazio"))?;
    world.out.push(format!("{} unload", world.car.t));
    world.car.t += 1;
    world.car.carrying = None;
    Ok(cid)
}

/* ==========================================================================
   PLANNER PRINCIPAL
   ========================================================================== */

pub fn plan_sequential(inst: &Instance) -> Result<Vec<String>> {
    // Inicialização
    let mut locs = HashMap::new();
    for (idx, stack) in inst.storage_stacks.iter().enumerate() {
        for &cid in stack { locs.insert(cid, Loc::InStorage(idx)); }
    }

    if inst.carriers.is_empty() { return Ok(Vec::new()); }
    let car0 = &inst.carriers[0];
    
    let mut world = WorldState {
        container_locs: locs,
        storage_stacks: inst.storage_stacks.clone(),
        car: CarState { id: car0.id, pos: car0.bl, dir: car0.dir, carrying: car0.carrying, t: 0 },
        out: Vec::new(),
    };
    world.out.push(format!("carrier {}", world.car.id));

    // Loop de Demandas
    for demand in &inst.demands {
        match *demand {
            Demand::Unload { dispatch_id, container_id, storage_id } => {
                // A) IR AO DISPATCH (SHIP -> DISPATCH) 
                let d = inst.dispatches.iter().find(|x| x.id == dispatch_id).unwrap();
                
                // O Carrier vai para a coordenada exata do Dispatch (Bottom-Left)
                drive_to_exact(&mut world, d.rect.x1, d.rect.y1)?;
                action_load(&mut world, container_id)?;

                // B) LEVAR À STORAGE 
                let s_idx = inst.storages.iter().position(|x| x.id == storage_id).unwrap();
                let s = &inst.storages[s_idx];

                if world.storage_stacks[s_idx].len() >= 2 {
                    return Err(anyhow!("Storage {} cheia!", storage_id));
                }

                // O Carrier alinha pelo X da storage e vai para o Y base
                drive_to_exact(&mut world, s.rect.x1, s.rect.y1)?;
                action_unload(&mut world)?;
                
                world.storage_stacks[s_idx].push(container_id);
                world.container_locs.insert(container_id, Loc::InStorage(s_idx));
            },
            Demand::Load { dispatch_id, container_id } => {
                let loc = *world.container_locs.get(&container_id).unwrap();
                if let Loc::InStorage(s_idx) = loc {
                    let s = &inst.storages[s_idx];
                    let stack = &world.storage_stacks[s_idx];

                    // --- Lógica de Reshuffle (Se soterrado) ---
                    if stack.len() > 1 && stack[0] == container_id {
                        let top_cid = stack[1];
                        
                        // 1. Pegar o de cima
                        drive_to_exact(&mut world, s.rect.x1, s.rect.y1)?;
                        action_load(&mut world, top_cid)?;
                        world.storage_stacks[s_idx].pop();
                        
                        // 2. Mover para vizinho livre
                        let free_idx = world.storage_stacks.iter().position(|st| st.len() < 2)
                            .ok_or_else(|| anyhow!("Sem espaço para reshuffle"))?;
                        let free_s = &inst.storages[free_idx];
                        
                        drive_to_exact(&mut world, free_s.rect.x1, free_s.rect.y1)?;
                        action_unload(&mut world)?;
                        
                        world.storage_stacks[free_idx].push(top_cid);
                        world.container_locs.insert(top_cid, Loc::InStorage(free_idx));
                    }
                    // ------------------------------------------

                    // 1. Pegar o alvo
                    drive_to_exact(&mut world, s.rect.x1, s.rect.y1)?;
                    action_load(&mut world, container_id)?;
                    world.storage_stacks[s_idx].retain(|&x| x != container_id);

                    // 2. Levar ao Dispatch
                    let d = inst.dispatches.iter().find(|x| x.id == dispatch_id).unwrap();
                    drive_to_exact(&mut world, d.rect.x1, d.rect.y1)?;
                    action_unload(&mut world)?;
                    
                    world.container_locs.insert(container_id, Loc::OnShip);
                }
            }
        }
    }
    Ok(world.out)
}