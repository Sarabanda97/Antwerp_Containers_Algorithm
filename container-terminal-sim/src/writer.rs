// This file contains functions for writing output data, such as the results of the simulation.

use std::fs;
use std::path::Path;

pub fn write_output(results: &str, output_path: &str) -> std::io::Result<()> {
    use std::fs::File;
    use std::io::Write;

    let mut file = File::create(output_path)?;
    file.write_all(results.as_bytes())?;
    Ok(())
}

pub fn write_solution(lines: &[String], path: &Path) -> std::io::Result<()> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let content = lines.join("\n");
    fs::write(path, content)?;
    Ok(())
}