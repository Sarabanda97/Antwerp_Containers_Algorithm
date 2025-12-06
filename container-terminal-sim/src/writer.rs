use crate::planner::path::Command;

pub fn write_solution(path: &str, cmds_by_carrier: &[(i32, Vec<Command>)]) -> std::io::Result<()> {
    use std::io::Write;
    let mut f = std::fs::File::create(path)?;

    for (cid, cmds) in cmds_by_carrier {
        writeln!(f, "carrier {}", cid)?;
        for cmd in cmds {
            match cmd {
                Command::Move { t, k } => writeln!(f, "{} move {}", t, k)?,
                Command::Face { t, dir } =>
                    writeln!(f, "{} face {:?}", t, dir)?,
                Command::Load { t } =>
                    writeln!(f, "{} load", t)?,
                Command::Unload { t } =>
                    writeln!(f, "{} unload", t)?,
            }
        }
    }
    Ok(())
}
