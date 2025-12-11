use container_terminal_sim::{
    parser::parse_instance,
    geometry::enrich_instance_geometry,
    planner::simple::plan_all_demands,
    writer::write_solution,
};

fn main() -> anyhow::Result<()> {

    // Helper to process one instance path -> output
    fn process(path_in: &str, path_out: &str) -> anyhow::Result<()> {
        let mut inst = parse_instance(path_in)?;
        enrich_instance_geometry(&mut inst);
        let cmds = plan_all_demands(&inst);
        write_solution(path_out, &[(0, cmds)])?;
        Ok(())
    }

    // process default toy instances
    println!("[RUN] processing toy_instance/toy.txt -> solutions/toy/solution_toy1.txt");
    process("../instances/toy_instance/toy.txt", "../solutions/toy/solution_toy1.txt")?;
    println!("[RUN] done toy_instance/toy.txt");

    println!("[RUN] processing instances/toy_b.txt -> solutions/toy/solution_toyB.txt");
    process("../instances/instances/toy_b.txt", "../solutions/toy/solution_toyB.txt")?;
    println!("[RUN] done instances/toy_b.txt");

    println!("[RUN] processing instances/toy_c.txt -> solutions/toy/solution_toyC.txt");
    process("../instances/instances/toy_c.txt", "../solutions/toy/solution_toyC.txt")?;
    println!("[RUN] done instances/toy_c.txt");

    Ok(())
}
