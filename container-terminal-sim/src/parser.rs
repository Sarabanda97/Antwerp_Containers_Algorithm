use anyhow::{anyhow, Context, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::collections::{HashMap, HashSet};
use crate::model::*;

/* ===================================================
 * Funções auxiliares
 * =================================================== */

/// remove comentários iniciados por `%` e faz trim
fn clean(s: &str) -> Option<String> {
    let s = s.split('%').next().unwrap_or("").trim();
    if s.is_empty() { None } else { Some(s.to_string()) }
}

/// verifica se a linha começa com número
fn peek_is_number(s: &str) -> bool {
    s.split_whitespace()
        .next()
        .map(|t| t.parse::<i32>().is_ok())
        .unwrap_or(false)
}

/// obtém próxima linha numérica (salta headers tipo "crane section")
fn next_number_line(lines: &Vec<String>, i: &mut usize) -> Option<String> {
    while *i < lines.len() && !peek_is_number(&lines[*i]) { *i += 1; }
    if *i < lines.len() {
        let v = lines[*i].clone();
        *i += 1;
        Some(v)
    } else {
        None
    }
}

/* ===================================================
 * Função principal
 * =================================================== */

pub fn parse_instance(path: &str) -> Result<Instance> {
    let file = File::open(path)
        .with_context(|| format!("Erro ao abrir o ficheiro {}", path))?;
    let lines: Vec<String> = BufReader::new(file)
        .lines()
        .filter_map(|l| l.ok())
        .filter_map(|l| clean(&l))
        .collect();

    let mut i = 0usize;
    let mut next = |i: &mut usize| -> Option<String> {
        if *i < lines.len() {
            let v = lines[*i].clone();
            *i += 1;
            Some(v)
        } else { None }
    };

    /* =====================
     * 1) Map (width height)
     * ===================== */
    while i < lines.len() && !peek_is_number(&lines[i]) { i += 1; }
    let first = next(&mut i).context("Ficheiro vazio ou sem dimensões")?;
    let mut it = first.split_whitespace();
    let width: i32 = it.next().context("Falta width")?.parse()?;
    let height: i32 = it.next().context("Falta height")?.parse()?;

    /* =====================
     * 2) Cranes + Dispatches
     * ===================== */
    let n_cranes_line = next_number_line(&lines, &mut i)
        .context("Falta número de cranes")?;
    let n_cranes: i32 = n_cranes_line.split_whitespace()
        .next().unwrap().parse()?;

    let mut cranes = Vec::new();
    let mut dispatches = Vec::new();
    let mut dispatch_ids_seen = HashSet::new();

    for _ in 0..n_cranes {
        let row = next(&mut i).context("Falta linha de crane")?;
        let nums: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;

        if nums.len() < 6 {
            return Err(anyhow!("Linha de crane inválida: {}", row));
        }

        let id = nums[0];
        let rect = Rect { x1: nums[1], y1: nums[2], x2: nums[3], y2: nums[4] };
        let nd = nums[5] as usize;

        let mut crane_dispatch_ids = Vec::new();
        for k in 0..nd {
            let base = 6 + 3 * k;
            if base + 2 >= nums.len() {
                return Err(anyhow!("Dispatch mal formatado na linha: {}", row));
            }
            let did = nums[base];
            let x = nums[base + 1];
            let y = nums[base + 2];
            if !dispatch_ids_seen.insert(did) {
                return Err(anyhow!("Dispatch id duplicado: {}", did));
            }
            let rect_d = Rect { x1: x, y1: y, x2: x + 3, y2: y + 1 };
            dispatches.push(Dispatch { id: did, crane_id: id, rect: rect_d });
            crane_dispatch_ids.push(did);
        }

        cranes.push(Crane { id, rect, dispatch_ids: crane_dispatch_ids });
    }

    /* =====================
     * 3) Storages
     * ===================== */
    let n_stor_line = next_number_line(&lines, &mut i)
        .context("Falta número de storages")?;
    let n_stor: i32 = n_stor_line.split_whitespace()
        .next().unwrap().parse()?;

    let mut storages = Vec::new();
    for _ in 0..n_stor {
        let row = next(&mut i).context("Falta linha de storage")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 3 {
            return Err(anyhow!("Linha de storage inválida: {}", row));
        }
        let id = v[0];
        let bl = Point { x: v[1], y: v[2] };
        let rect = Rect { x1: bl.x, y1: bl.y, x2: bl.x + 1, y2: bl.y + 3 };
        storages.push(Storage { id, rect });
    }

    /* =====================
     * 4) Carriers
     * ===================== */
    let n_car_line = next_number_line(&lines, &mut i)
        .context("Falta número de carriers")?;
    let n_car: i32 = n_car_line.split_whitespace()
        .next().unwrap().parse()?;

    let mut carriers = Vec::new();
    for _ in 0..n_car {
        let row = next(&mut i).context("Falta linha de carrier")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 4 {
            return Err(anyhow!("Linha de carrier inválida: {}", row));
        }
        carriers.push(Carrier {
            id: v[0],
            assigned_crane: v[1],
            bl: Point { x: v[2], y: v[3] },
            dir: Direction::Down,
            carrying: None,
            size: (4, 8),
        });
    }

    /* =====================
     * 5) Containers iniciais
     * ===================== */
    let n_cont_line = next_number_line(&lines, &mut i)
        .context("Falta número de containers iniciais")?;
    let n_cont: i32 = n_cont_line.split_whitespace()
        .next().unwrap().parse()?;

    let mut storage_stacks: Vec<Vec<Id>> = vec![Vec::new(); storages.len()];
    for _ in 0..n_cont {
        let row = next(&mut i).context("Falta linha de container")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 2 {
            return Err(anyhow!("Linha de container inválida: {}", row));
        }
        let cid = v[0];
        let sid = v[1];
        let idx = storages.iter().position(|s| s.id == sid)
            .with_context(|| format!("Storage id inválido: {}", sid))?;
        storage_stacks[idx].push(cid);
    }

    // valida stacks
    for (sid, st) in storage_stacks.iter().enumerate() {
        if st.len() > 2 {
            return Err(anyhow!("Storage {} tem mais de 2 contentores", sid));
        }
    }

    /* =====================
     * 6) Demands
     * ===================== */
    let mut demands = Vec::new();
    while i < lines.len() {
        let l = lines[i].clone();
        i += 1;

        let toks: Vec<&str> = l.split_whitespace().collect();
        if toks.is_empty() { continue; }
        let op = toks[0].to_lowercase();

        match op.as_str() {
            "unload" => {
                if toks.len() < 4 {
                    return Err(anyhow!("Linha unload inválida: {}", l));
                }
                let dispatch_id: i32 = toks[1].parse()?;
                let container_id: i32 = toks[2].parse()?;
                let storage_id: i32 = toks[3].parse()?;

                if !dispatches.iter().any(|d| d.id == dispatch_id) {
                    return Err(anyhow!("Dispatch id {} não existe (linha: {})", dispatch_id, l));
                }
                demands.push(Demand::Unload { dispatch_id, container_id, storage_id });
            }
            "load" => {
                if toks.len() < 3 {
                    return Err(anyhow!("Linha load inválida: {}", l));
                }
                let dispatch_id: i32 = toks[1].parse()?;
                let container_id: i32 = toks[2].parse()?;

                if !dispatches.iter().any(|d| d.id == dispatch_id) {
                    return Err(anyhow!("Dispatch id {} não existe (linha: {})", dispatch_id, l));
                }
                demands.push(Demand::Load { dispatch_id, container_id });
            }
            _ => { /* ignora headers: 'demand section', 'ship 0', etc. */ }
        }
    }

    /* =====================
     * Construir Instance
     * ===================== */
    Ok(Instance {
        width,
        height,
        cranes,
        dispatches,
        storages,
        carriers,
        storage_stacks,
        demands,
    })
}
