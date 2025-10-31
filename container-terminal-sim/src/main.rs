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
    // default relative paths (from inside container-terminal-sim/)
    let inst_path = r"..\instances\toy_instance\toy.txt";
    let mut out_path = r"..\solutions\toy\solution_toy.txt";

    // optional arg "first" -> keep only 1 demand
    let args: Vec<String> = env::args().collect();
    let first_only = args.iter().any(|a| a == "first");

    // 1) parse
    let mut inst = parser::parse_instance(inst_path)?;
    println!("{}", inst);

    // 2) maybe trim demands
    if first_only {
        if !inst.demands.is_empty() {
            inst.demands = inst.demands.into_iter().take(1).collect();
        }
        out_path = r"..\solutions\toy\solution_toy_first.txt";
    }

    // 3) plan – use the dynamic one
    let plan_lines = planner::plan_sequential(&inst);
    // if you want to test the old fixed one:
    // let plan_lines = planner_simple::plan_simple(&inst);

    // 4) group by carrier and STRIP carrier id from lines (checker format)
    let mut final_lines: Vec<String> = Vec::new();

    if !inst.carriers.is_empty() {
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
            // fallback: first carrier
            let default_cid = inst.carriers[0].id;
            by_carrier.entry(default_cid).or_default().push(l.clone());
        }

        // emit in instance order
        for c in &inst.carriers {
            // checker wants just the id on its own line, e.g. "0"
            final_lines.push(format!("carrier {}", c.id));
            if let Some(lines) = by_carrier.get(&c.id) {
                for l in lines {
                    // l = "0 0 face down"  -> we need "0 face down"
                    let mut toks = l.split_whitespace();
                    let t = toks.next().unwrap();      // timestamp
                    let _cid = toks.next().unwrap();   // drop carrier id
                    let rest = toks.collect::<Vec<_>>().join(" ");
                    final_lines.push(format!("{} {}", t, rest));
                }
            }
        }
    }

    // 5) make sure folder exists
    if let Some(parent) = Path::new(out_path).parent() {
        fs::create_dir_all(parent)?;
    }

    // 6) write solution
    writer::write_solution(&final_lines, Path::new(out_path))?;
    println!("Wrote {}", out_path);

    Ok(())
}