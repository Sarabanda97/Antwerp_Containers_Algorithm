use container_terminal_sim::{
    parser::parse_instance,
    geometry::enrich_instance_geometry,
    planner::simple::plan_all_demands,
    writer::write_solution,
};

fn main() -> anyhow::Result<()> {
    let mut inst = parse_instance("../instances/toy_instance/toy.txt")?;
    enrich_instance_geometry(&mut inst);

    let cmds = plan_all_demands(&inst);

    write_solution("../solutions/toy/solution_toy1.txt", &[(0, cmds)])?;
    Ok(())
}
