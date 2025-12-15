use container_terminal_sim::{
    parser::parse_instance,
    geometry::enrich_instance_geometry,
    planner::simple::plan_all_demands,
    writer::write_solution,
};
use std::path::Path;
use std::fs;

fn main() -> anyhow::Result<()> {

    // Helper to process one instance path -> output
    fn process(path_in: &Path, path_out: &Path) -> anyhow::Result<()> {
        let mut inst = parse_instance(path_in.to_str().unwrap())?;
        enrich_instance_geometry(&mut inst);
        let cmds = plan_all_demands(&inst);
        write_solution(path_out.to_str().unwrap(), &[(0, cmds)])?;
        Ok(())
    }

    // list of basic instances to process (relative to container-terminal-sim)
    let instances = vec![
        "../instances/basic_instances/small_basic_00.txt",
        "../instances/basic_instances/small_basic_01.txt",
        "../instances/basic_instances/tiny_basic_00.txt",
        "../instances/basic_instances/tiny_basic_01.txt",
    ];

    // ensure output dir exists
    let out_dir = Path::new("../solutions/basic_instances");
    if !out_dir.exists() {
        fs::create_dir_all(out_dir)?;
    }

    for inst_path in instances {
        let in_path = Path::new(inst_path);
        let stem = in_path.file_stem().and_then(|s| s.to_str()).unwrap_or("instance");
        let out_file_name = format!("solution_{}.txt", stem);
        let out_path = out_dir.join(out_file_name);

        println!("[RUN] processing {} -> {}", in_path.display(), out_path.display());
        if let Err(e) = process(in_path, &out_path) {
            eprintln!("[ERROR] processing {}: {}", in_path.display(), e);
            // continue with next instance
        } else {
            println!("[DONE] {}", out_path.display());
        }
    }

    Ok(())
}
