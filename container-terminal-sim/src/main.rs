mod model;
mod parser;
mod planner;
mod writer;

use std::path::Path;

fn main() -> anyhow::Result<()> {
    let inst_path = "../instances/toy_instance/toy.txt";
    let inst = parser::parse_instance(inst_path)?;

    let plan = planner::plan_sequential(&inst)?;
    let out_path = Path::new("../solutions/toy/solution_DEMO_toy.txt");

    writer::write_solution(&plan, out_path)?;
    // também mostra no stdout para debug
    for line in &plan { println!("{}", line); }

    Ok(())
}
