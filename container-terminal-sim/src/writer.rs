use crate::model::Direction;
use crate::planner::path::Command;

pub fn write_solution(
    path: &str,
    carrier_plans: &[(i32, Vec<Command>)],
) -> anyhow::Result<()> {
    use std::io::Write;

    let mut file = std::fs::File::create(path)?;

    for (carrier_id, cmds) in carrier_plans {
        writeln!(file, "carrier {}", carrier_id)?;

        for cmd in cmds {
            match cmd {
                Command::Move { t, k } => {
                    writeln!(file, "{} move {}", t, k)?;
                }
                Command::Face { t, dir } => {
                    let dir_str = match dir {
                        Direction::Up    => "up",
                        Direction::Down  => "down",
                        Direction::Left  => "left",
                        Direction::Right => "right",
                    };
                    writeln!(file, "{} face {}", t, dir_str)?;
                }
                Command::Load { t } => {
                    writeln!(file, "{} load", t)?;
                }
                Command::Unload { t } => {
                    writeln!(file, "{} unload", t)?;
                }
            }
        }
    }

    Ok(())
}
