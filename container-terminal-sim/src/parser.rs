use anyhow::{anyhow, Context, Result};
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{BufRead, BufReader};
use crate::model::*;

/* ===========================
 * Auxiliares
 * =========================== */

fn clean(s: &str) -> Option<String> {
    let s = s.split('%').next().unwrap_or("").trim();
    if s.is_empty() { None } else { Some(s.to_string()) }
}

fn peek_is_number(s: &str) -> bool {
    s.split_whitespace()
        .next()
        .map(|t| t.parse::<i32>().is_ok())
        .unwrap_or(false)
}

fn next_number_line(lines: &Vec<String>, i: &mut usize) -> Option<String> {
    while *i < lines.len() && !peek_is_number(&lines[*i]) { *i += 1; }
    if *i < lines.len() {
        let v = lines[*i].clone();
        *i += 1;
        Some(v)
    } else { None }
}

fn rect_within(a: &Rect, b: &Rect) -> bool {
    a.x1 >= b.x1 && a.y1 >= b.y1 && a.x2 <= b.x2 && a.y2 <= b.y2
}

fn rect_intersects(a: &Rect, b: &Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}

/* ===========================
 * Parser principal
 * =========================== */

pub fn parse_instance(path: &str) -> Result<Instance> {
    let file = File::open(path)
        .with_context(|| format!("Erro ao abrir o ficheiro {}", path))?;
    let lines: Vec<String> = BufReader::new(file)
        .lines()
        .filter_map(|l| l.ok())
        .filter_map(|l| clean(&l))
        .collect();

    let mut i = 0usize;
    let next = |i: &mut usize| -> Option<String> {
        if *i < lines.len() { let v = lines[*i].clone(); *i += 1; Some(v) } else { None }
    };

    /* ========== 1) MAPA ========== */
    while i < lines.len() && !peek_is_number(&lines[i]) { i += 1; }
    let first = next(&mut i).context("Ficheiro vazio ou sem dimensões")?;
    let mut it = first.split_whitespace();
    let width: i32  = it.next().context("Falta width")?.parse()?;
    let height: i32 = it.next().context("Falta height")?.parse()?;
    if width <= 0 || height <= 0 {
        return Err(anyhow!("Dimensões do mapa inválidas: {}x{}", width, height));
    }
    let map_rect = Rect { x1: 0, y1: 0, x2: width, y2: height };

    /* ========== 2) CRANES + DISPATCHES ========== */
    let n_cranes_line = next_number_line(&lines, &mut i)
        .context("Falta número de cranes")?;
    let n_cranes: i32 = n_cranes_line.split_whitespace().next().unwrap().parse()?;

    let mut cranes: Vec<Crane> = Vec::new();
    let mut dispatches: Vec<Dispatch> = Vec::new();
    let mut crane_ids_seen = HashSet::new();
    let mut dispatch_ids_seen = HashSet::new();

    for _ in 0..n_cranes {
        let row = next(&mut i).context("Falta linha de crane")?;
        let nums: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;

        if nums.len() < 6 { return Err(anyhow!("Linha de crane inválida: {}", row)); }

        let id = nums[0];
        if !crane_ids_seen.insert(id) {
            return Err(anyhow!("Crane id duplicado: {}", id));
        }
        let rect = Rect { x1: nums[1], y1: nums[2], x2: nums[3], y2: nums[4] };
        if !rect_within(&rect, &map_rect) {
            return Err(anyhow!("Crane {} fora do mapa: rect=({},{},{},{})",
                id, rect.x1, rect.y1, rect.x2, rect.y2));
        }

        let nd = nums[5] as usize;
        let mut crane_dispatch_ids = Vec::new();
        let mut this_dispatch_rects: Vec<Rect> = Vec::new();

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

            // dispatch dentro do mapa
            if !rect_within(&rect_d, &map_rect) {
                return Err(anyhow!("Dispatch {} fora do mapa: rect=({},{},{},{})",
                    did, rect_d.x1, rect_d.y1, rect_d.x2, rect_d.y2));
            }
            // dispatch dentro da crane section
            if !rect_within(&rect_d, &rect) {
                return Err(anyhow!(
                    "Dispatch {} não está contido no rect da grua {}",
                    did, id
                ));
            }
            // sem overlap com outras dispatches da mesma grua
            for prev in &this_dispatch_rects {
                if rect_intersects(prev, &rect_d) {
                    return Err(anyhow!(
                        "Dispatch {} sobrepõe-se a outro dispatch desta grua {}",
                        did, id
                    ));
                }
            }
            this_dispatch_rects.push(rect_d);

            dispatches.push(Dispatch {
                id: did,
                crane_id: id,
                rect: rect_d,
                staging_bl: None,
                staging_dir: None,
            });
            crane_dispatch_ids.push(did);
        }

        cranes.push(Crane { id, rect, dispatch_ids: crane_dispatch_ids });
    }

    /* ========== 3) STORAGES ========== */
    let n_stor_line = next_number_line(&lines, &mut i)
        .context("Falta número de storages")?;
    let n_stor: i32 = n_stor_line.split_whitespace().next().unwrap().parse()?;

    let mut storages: Vec<Storage> = Vec::new();
    let mut storage_ids_seen = HashSet::new();

    for _ in 0..n_stor {
        let row = next(&mut i).context("Falta linha de storage")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 3 { return Err(anyhow!("Linha de storage inválida: {}", row)); }

        let id = v[0];
        if !storage_ids_seen.insert(id) {
            return Err(anyhow!("Storage id duplicado: {}", id));
        }
        let bl = Point { x: v[1], y: v[2] };
        let rect = Rect { x1: bl.x, y1: bl.y, x2: bl.x + 1, y2: bl.y + 3 }; // 2×4
        if !rect_within(&rect, &map_rect) {
            return Err(anyhow!("Storage {} fora do mapa: rect=({},{},{},{})",
                id, rect.x1, rect.y1, rect.x2, rect.y2));
        }

        storages.push(Storage {
            id,
            rect,
            staging_bl: None,
            staging_dir: None,
        });
    }

    /* ========== 4) CARRIERS ========== */
    let n_car_line = next_number_line(&lines, &mut i)
        .context("Falta número de carriers")?;
    let n_car: i32 = n_car_line.split_whitespace().next().unwrap().parse()?;

    let mut carriers: Vec<Carrier> = Vec::new();
    let crane_id_exists: HashSet<i32> = cranes.iter().map(|c| c.id).collect();

    for _ in 0..n_car {
        let row = next(&mut i).context("Falta linha de carrier")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 4 { return Err(anyhow!("Linha de carrier inválida: {}", row)); }

        let id = v[0];
        let assigned_crane = v[1];
        if !crane_id_exists.contains(&assigned_crane) {
            return Err(anyhow!("Carrier referencia crane inexistente: {}", assigned_crane));
        }

        let bl = Point { x: v[2], y: v[3] };

        carriers.push(Carrier {
            id,
            assigned_crane,
            bl,
            dir: Direction::Down,
            carrying: None,
            size: (4, 8),
        });
    }

    /* ========== 5) CONTAINERS INICIAIS ========== */
    let n_cont_line = next_number_line(&lines, &mut i)
        .context("Falta número de containers iniciais")?;
    let n_cont: i32 = n_cont_line.split_whitespace().next().unwrap().parse()?;

    let mut storage_index_by_id: HashMap<i32, usize> = HashMap::new();
    for (idx, s) in storages.iter().enumerate() {
        storage_index_by_id.insert(s.id, idx);
    }

    let mut storage_stacks: Vec<Vec<Id>> = vec![Vec::new(); storages.len()];
    for _ in 0..n_cont {
        let row = next(&mut i).context("Falta linha de container")?;
        let v: Vec<i32> = row.split_whitespace()
            .map(|t| t.parse::<i32>())
            .collect::<Result<_, _>>()?;
        if v.len() < 2 { return Err(anyhow!("Linha de container inválida: {}", row)); }

        let cid = v[0];
        let sid = v[1];
        let idx = storage_index_by_id.get(&sid)
            .copied()
            .with_context(|| format!("Storage id inválido: {}", sid))?;
        storage_stacks[idx].push(cid);
    }

    for (k, st) in storage_stacks.iter().enumerate() {
        if st.len() > 2 {
            return Err(anyhow!("Storage {} tem mais de 2 contentores", k));
        }
    }

    /* ========== 6) DEMAND SECTION ========== */
    let mut demands: Vec<Demand> = Vec::new();
    let mut ships: Vec<ShipBlock> = Vec::new();
    let mut total_new_containers: Option<i32> = None;
    let mut current_crane: Option<i32> = None;

    while i < lines.len() {
        let l = lines[i].clone(); i += 1;
        let toks: Vec<&str> = l.split_whitespace().collect();
        if toks.is_empty() { continue; }

        match toks[0].to_lowercase().as_str() {
            "demand" if toks.len() >= 2 && toks[1].eq_ignore_ascii_case("section") => {
                let saved_i = i;
                if let Some(nline) = next_number_line(&lines, &mut i) {
                    let n: i32 = nline.split_whitespace().next().unwrap().parse()?;
                    total_new_containers = Some(n);
                } else {
                    i = saved_i;
                }
            }

            "demand" if toks.len() >= 3 && toks[1].eq_ignore_ascii_case("crane") => {
                let cid: i32 = toks[2].parse()?;
                if !crane_id_exists.contains(&cid) {
                    return Err(anyhow!("'demand crane {}' refere grua inexistente", cid));
                }
                current_crane = Some(cid);

                let saved_i = i;
                if let Some(nline) = next_number_line(&lines, &mut i) {
                    let mut it = nline.split_whitespace();
                    if let Some(n_tok) = it.next() {
                        let nships: usize = n_tok.parse()?;
                        let ship_ids: Vec<i32> = it.map(|t| t.parse::<i32>())
                                                   .collect::<Result<_, _>>()?;
                        if ship_ids.len() != nships {
                            return Err(anyhow!(
                                "Esperava {} ship ids, obtive {} na linha '{}'",
                                nships, ship_ids.len(), nline
                            ));
                        }
                        // ship_ids não usados por agora
                    }
                } else {
                    i = saved_i;
                }
            }

            "ship" if toks.len() >= 2 => {
                let ship_id: i32 = toks[1].parse()?;
                let mut block = ShipBlock {
                    ship_id,
                    crane_id: current_crane,
                    operations: vec![],
                };

                let mline = next_number_line(&lines, &mut i)
                    .context(format!("Falta número de operações para ship {}", ship_id))?;
                let m_ops: usize = mline.split_whitespace().next().unwrap().parse()?;

                for _ in 0..m_ops {
                    let op_line = next(&mut i).context("Falta linha de operação em ship")?;
                    let tk: Vec<&str> = op_line.split_whitespace().collect();
                    if tk.is_empty() {
                        return Err(anyhow!("Linha de operação vazia: {}", op_line));
                    }

                    match tk[0].to_lowercase().as_str() {
                        "unload" => {
                            if tk.len() < 4 {
                                return Err(anyhow!("Linha unload inválida: {}", op_line));
                            }
                            let dispatch_id: i32 = tk[1].parse()?;
                            let container_id: i32 = tk[2].parse()?;
                            let storage_id: i32 = tk[3].parse()?;

                            if !dispatches.iter().any(|d| d.id == dispatch_id) {
                                return Err(anyhow!("Dispatch id {} não existe ({})",
                                    dispatch_id, op_line));
                            }
                            if !storage_index_by_id.contains_key(&storage_id) {
                                return Err(anyhow!("Storage id {} não existe ({})",
                                    storage_id, op_line));
                            }

                            let d = Demand::Unload { dispatch_id, container_id, storage_id };
                            demands.push(d.clone());
                            block.operations.push(d);
                        }
                        "load" => {
                            if tk.len() < 3 {
                                return Err(anyhow!("Linha load inválida: {}", op_line));
                            }
                            let dispatch_id: i32 = tk[1].parse()?;
                            let container_id: i32 = tk[2].parse()?;

                            if !dispatches.iter().any(|d| d.id == dispatch_id) {
                                return Err(anyhow!("Dispatch id {} não existe ({})",
                                    dispatch_id, op_line));
                            }

                            let d = Demand::Load { dispatch_id, container_id };
                            demands.push(d.clone());
                            block.operations.push(d);
                        }
                        other => {
                            return Err(anyhow!("Operação desconhecida '{}' na linha '{}'",
                                other, op_line));
                        }
                    }
                }
                ships.push(block);
            }

            "unload" | "load" => {
                let op = toks[0].to_lowercase();
                if op == "unload" {
                    if toks.len() < 4 {
                        return Err(anyhow!("Linha unload inválida: {}", l));
                    }
                    let dispatch_id: i32 = toks[1].parse()?;
                    let container_id: i32 = toks[2].parse()?;
                    let storage_id: i32 = toks[3].parse()?;
                    if !dispatches.iter().any(|d| d.id == dispatch_id) {
                        return Err(anyhow!("Dispatch id {} não existe (linha: {})",
                            dispatch_id, l));
                    }
                    if !storage_index_by_id.contains_key(&storage_id) {
                        return Err(anyhow!("Storage id {} não existe (linha: {})",
                            storage_id, l));
                    }
                    demands.push(Demand::Unload { dispatch_id, container_id, storage_id });
                } else {
                    if toks.len() < 3 {
                        return Err(anyhow!("Linha load inválida: {}", l));
                    }
                    let dispatch_id: i32 = toks[1].parse()?;
                    let container_id: i32 = toks[2].parse()?;
                    if !dispatches.iter().any(|d| d.id == dispatch_id) {
                        return Err(anyhow!("Dispatch id {} não existe (linha: {})",
                            dispatch_id, l));
                    }
                    demands.push(Demand::Load { dispatch_id, container_id });
                }
            }

            _ => { /* ignora outros rótulos livres */ }
        }
    }

    /* ========== 7) Construir Instance ========== */
    Ok(Instance {
        width,
        height,
        cranes,
        dispatches,
        storages,
        carriers,
        storage_stacks,
        demands,
        total_new_containers,
        ships,
        yard_rect: None,
    })
}
