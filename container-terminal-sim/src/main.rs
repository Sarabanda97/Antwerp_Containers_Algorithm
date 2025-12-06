mod model;
mod parser;
mod geometry;
mod state;
mod planner;
mod writer;

use parser::parse_instance;
use geometry::enrich_instance_geometry;
use planner::simple::plan_all_demands;
use writer::write_solution;

fn main() -> anyhow::Result<()> {
    let mut inst = parse_instance("../instances/toy_instance/toy.txt")?;
    enrich_instance_geometry(&mut inst);
    
    println!("================ STAGING DEBUG ================");
    println!("YARD RECT: {:?}", inst.yard_rect);

    println!("\nSTORAGES:");
    for s in &inst.storages {
        println!(
            "id {} | rect {:?} | staging {:?} dir {:?}",
            s.id, s.rect, s.staging_bl, s.staging_dir
        );
    }

    println!("\nDISPATCHES:");
    for d in &inst.dispatches {
        println!(
            "id {} | rect {:?} | staging {:?} dir {:?}",
            d.id, d.rect, d.staging_bl, d.staging_dir
        );
    }
    println!("===============================================");

    let cmds = plan_all_demands(&inst);

    // para já só 1 carrier, id 0
    write_solution("../solutions/toy/solution_toy1.txt", &[(0, cmds)])?;

    Ok(())
    
}
