use container_terminal_sim::{
    parser::parse_instance,
    geometry::enrich_instance_geometry,
    planner::simple::plan_all_demands_multi,
    writer::write_solution,
};
use std::path::Path;
use std::fs;
use std::env;

fn main() -> anyhow::Result<()> {
    fn process(path_in: &Path, path_out: &Path) -> anyhow::Result<()> {
        let mut inst = parse_instance(path_in.to_str().unwrap())?;
        enrich_instance_geometry(&mut inst);
        
        // USA A FUNÇÃO MULTI
        let plans = plan_all_demands_multi(&inst);
        write_solution(path_out.to_str().unwrap(), &plans)?;
        Ok(())
    }

    // CLI usage:
    //   cargo run --release -- <instance_path> <solution_out_path>
    // If no args are given, we generate solutions for the target instances.
    let args: Vec<String> = env::args().collect();
    if args.len() >= 3 {
        let in_path = Path::new(&args[1]);
        let out_path = Path::new(&args[2]);
        println!("[RUN] processing {} -> {}", in_path.display(), out_path.display());
        process(in_path, out_path)?;
        println!("[DONE] {}", out_path.display());
        return Ok(());
    }

    let instances = vec![
        "../instances/basic_instances/normal_basic_01.txt",
        "../instances/basic_instances/large_basic_01.txt",
    ];

    let out_dir = Path::new("../solutions/basic_instances");
    if !out_dir.exists() { fs::create_dir_all(out_dir)?; }

    for inst_path in instances {
        let in_path = Path::new(inst_path);
        let stem = in_path.file_stem().and_then(|s| s.to_str()).unwrap_or("instance");
        let out_file_name = format!("solution_{}.txt", stem);
        let out_path = out_dir.join(out_file_name);

        println!("[RUN] processing {} -> {}", in_path.display(), out_path.display());
        if let Err(e) = process(in_path, &out_path) {
            eprintln!("[ERROR] processing {}: {}", in_path.display(), e);
        } else {
            println!("[DONE] {}", out_path.display());
        }
    }
    Ok(())
}