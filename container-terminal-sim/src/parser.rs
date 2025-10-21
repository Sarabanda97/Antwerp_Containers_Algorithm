use anyhow::{Context, Result};
use std::fs::File;
use std::io::{BufRead, BufReader};
use crate::model::*;

fn clean(s:&str)->Option<String>{
    let s = s.split('%').next().unwrap_or("").trim();
    if s.is_empty() { None } else { Some(s.to_string()) }
}

fn peek_is_number(s: &str) -> bool {
    s.split_whitespace().next().map(|t| t.parse::<i32>().is_ok()).unwrap_or(false)
}

pub fn parse_instance(path:&str) -> Result<Instance> {
    let f = File::open(path).with_context(|| format!("a abrir {}", path))?;
    let mut lines: Vec<String> = BufReader::new(f)
        .lines().filter_map(|l| l.ok())
        .filter_map(|l| clean(&l)).collect();
    let mut i=0usize;
    let next = |i:&mut usize| { let v = lines.get(*i).cloned().unwrap_or_default(); *i+=1; v };

    // 1) mapa
    // skip until we find a line starting with a number
    while i < lines.len() && !peek_is_number(&lines[i]) { i += 1; }
    let first = next(&mut i);
    let mut it = first.split_whitespace();
    let width:i32 = it.next().context("missing width")?.parse()?;
    let height:i32 = it.next().context("missing height")?.parse()?;

    // helper to get next integer line (skip headers)
    let mut next_number_line = |i:&mut usize| -> String {
        while *i < lines.len() && !peek_is_number(&lines[*i]) { *i += 1; }
        next(i)
    };

    // 2) cranes
    let n_cranes_line = next_number_line(&mut i);
    let n_cranes:i32 = n_cranes_line.split_whitespace().next().unwrap_or("0").parse().unwrap_or(0);
    let mut cranes=Vec::new();
    for _ in 0..n_cranes {
        let row = next(&mut i);
        let nums:Vec<i32> = row.split_whitespace().map(|t| t.parse().unwrap()).collect();
        // id x1 y1 x2 y2 nd d1x d1y ...
        let id=nums[0];
        let rect=(nums[1],nums[2],nums[3],nums[4]);
        let nd=nums[5] as usize;
        let mut dispatches=Vec::new();
        for k in 0..nd { dispatches.push((nums[6+2*k], nums[6+2*k+1])); }
        cranes.push(Crane{ id, rect, dispatch_positions:dispatches });
    }

    // 3) storages
    let n_stor_line = next_number_line(&mut i);
    let n_stor:i32 = n_stor_line.split_whitespace().next().unwrap_or("0").parse().unwrap_or(0);
    let mut storages=Vec::new();
    for _ in 0..n_stor {
        let row = next(&mut i);
        let v:Vec<i32>=row.split_whitespace().map(|t| t.parse().unwrap()).collect();
        storages.push(Storage{ id:v[0], bl:(v[1],v[2]) });
    }

    // 4) carriers
    let n_car_line = next_number_line(&mut i);
    let n_car:i32 = n_car_line.split_whitespace().next().unwrap_or("0").parse().unwrap_or(0);
    let mut carriers=Vec::new();
    for _ in 0..n_car {
        let row = next(&mut i);
        let v:Vec<i32>=row.split_whitespace().map(|t| t.parse().unwrap()).collect();
        carriers.push(Carrier{ id:v[0], crane_id:v[1], bl:(v[2],v[3]) });
    }

    // 5) containers iniciais
    let n_cont_line = next_number_line(&mut i);
    let n_cont:i32 = n_cont_line.split_whitespace().next().unwrap_or("0").parse().unwrap_or(0);
    let mut containers_init=Vec::new();
    for _ in 0..n_cont {
        let row = next(&mut i);
        let v:Vec<i32>=row.split_whitespace().map(|t| t.parse().unwrap()).collect();
        if v.len() >= 2 { containers_init.push((v[0], v[1])); }
    }

    // 6) demands (até ao fim) — aceita "unload" / "load" case-insensitive
    let mut demands=Vec::new();
    while i<lines.len() {
        let l = next(&mut i);
        let toks:Vec<&str> = l.split_whitespace().collect();
        if toks.is_empty() { continue; }
        let op = toks[0].to_lowercase();
        match op.as_str() {
            "load" => {
                if toks.len() >= 3 {
                    let crane_id = toks[1].parse()?;
                    let container_id = toks[2].parse()?;
                    demands.push(Demand::Load{ crane_id, container_id });
                }
            }
            "unload" => {
                if toks.len() >= 4 {
                    let crane_id: i32 = toks[1].parse()?;
                    let container_id: i32 = toks[2].parse()?;
                    let storage_id: i32 = toks[3].parse()?;
                    demands.push(Demand::Unload{ crane_id, container_id, storage_id });
                }
            }
            _ => {} // ignora outras linhas
        }
    }

    Ok(Instance{ width,height, cranes, storages, carriers, containers_init, demands })
}
