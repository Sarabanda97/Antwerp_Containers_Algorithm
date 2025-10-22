mod model;
mod parser;
mod planner;
mod planner_simple;
mod writer;

use anyhow::Result;
use std::fs;
use std::path::Path;
use std::env;

fn main() -> Result<()> {
    // instancia/outputs (relativos ao package)
    let inst_path = r"..\instances\toy_instance\toy.txt";
    let mut out_path  = r"..\solutions\toy\solution_toy.txt";

    // aceita argumento "first" para gerar só a 1ª demanda
    let args: Vec<String> = env::args().collect();
    let first_only = args.iter().any(|a| a == "first");

    // parsing
    let mut inst = parser::parse_instance(inst_path)?;
    println!("{}", inst); // inspeciona o parser

    if first_only {
        if !inst.demands.is_empty() {
            inst.demands = inst.demands.into_iter().take(1).collect();
        }
        out_path = r"..\solutions\toy\solution_toy_first.txt";
    }

    // planeador simple (step by step)
    let plan_lines = planner_simple::plan_simple(&inst);

    // Group plan_lines by carrier and emit "carrier <id>" blocks expected by the checker
    let mut final_lines: Vec<String> = Vec::new();

    if inst.carriers.is_empty() {
        // no carriers -> nothing
    } else {
        // build a map from carrier id -> Vec<lines>
        use std::collections::HashMap;
        let mut by_carrier: HashMap<i32, Vec<String>> = HashMap::new();
        for l in &plan_lines {
            let toks: Vec<&str> = l.split_whitespace().collect();
            if toks.len() >= 2 {
                if let Ok(cid) = toks[1].parse::<i32>() {
                    by_carrier.entry(cid).or_default().push(l.clone());
                    continue;
                }
            }
            // fallback: put into first carrier
            let default_cid = inst.carriers[0].id;
            by_carrier.entry(default_cid).or_default().push(l.clone());
        }

        // emit blocks in order of carriers as listed in the instance
        for c in &inst.carriers {
            final_lines.push(format!("carrier {}", c.id));
            if let Some(lines) = by_carrier.get(&c.id) {
                final_lines.extend(lines.clone());
            }
        }
    }

    if let Some(parent) = Path::new(out_path).parent() {
        fs::create_dir_all(parent)?;
    }
    // use writer helper to persist
    writer::write_solution(&final_lines, Path::new(out_path))?;
    println!("Wrote {}", out_path);

    Ok(())
}
