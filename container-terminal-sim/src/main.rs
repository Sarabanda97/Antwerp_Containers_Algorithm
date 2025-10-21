mod model;
mod parser;
mod planner;

use anyhow::Result;
use std::fs;
use std::path::Path;

fn main() -> Result<()> {
    let inst_path = r"..\instances\toy_instance\toy.txt";
let out_path  = r"..\solutions\toy\solution_toy.txt";

    let inst = parser::parse_instance(inst_path)?;
    println!("{}", inst);

    let plan_lines = planner::plan_sequential(&inst);

    if let Some(parent) = Path::new(out_path).parent() {
        fs::create_dir_all(parent)?;
    }
    fs::write(out_path, plan_lines.join("\n"))?;
    println!("Wrote {}", out_path);

    Ok(())
}
