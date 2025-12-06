use crate::model::*;
use crate::state::CarrierState;

#[derive(Clone, Debug)]
pub enum Command {
    Move { t: i32, k: i32 },
    Face { t: i32, dir: Direction },
    Load { t: i32 },
    Unload { t: i32 },
}

// rect do carrier dado (bl, dir)
fn carrier_rect(bl: Point, dir: Direction) -> Rect {
    match dir {
        Direction::Up | Direction::Down => Rect {
            x1: bl.x,
            y1: bl.y,
            x2: bl.x + 3,
            y2: bl.y + 7,
        },
        Direction::Left | Direction::Right => Rect {
            x1: bl.x,
            y1: bl.y,
            x2: bl.x + 7,
            y2: bl.y + 3,
        },
    }
}

fn rect_intersects(a: &Rect, b: &Rect) -> bool {
    !(a.x2 < b.x1 || b.x2 < a.x1 || a.y2 < b.y1 || b.y2 < a.y1)
}

fn in_yard(inst: &Instance, bl: Point, dir: Direction) -> bool {
    if let Some(yard) = inst.yard_rect {
        let r = carrier_rect(bl, dir);
        rect_intersects(&r, &yard)
    } else {
        false
    }
}

fn face_to(c: &mut CarrierState, dir: Direction, cmds: &mut Vec<Command>) {
    if c.dir == dir { return; }
    let t = c.time;
    cmds.push(Command::Face { t, dir });
    c.time += 1;
    c.dir = dir;
    // simplificação: assumimos que a bottom-left não muda no face
}

fn move_forward(c: &mut CarrierState, steps: i32, cmds: &mut Vec<Command>) {
    if steps == 0 { return; }
    let t = c.time;
    cmds.push(Command::Move { t, k: steps });
    // **IMPORTANTE**: tempo cresce pelo valor absoluto,
    // mesmo quando andamos para trás (k < 0)
    c.time += steps.abs();

    match c.dir {
        Direction::Up    => { c.bl.y += steps; }
        Direction::Down  => { c.bl.y -= steps; }
        Direction::Left  => { c.bl.x -= steps; }
        Direction::Right => { c.bl.x += steps; }
    }
}

pub fn go_to_pose(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let start_in_yard = in_yard(inst, c.bl, c.dir);
    let target_rect   = carrier_rect(target_bl, target_dir);
    let target_in_yard = if let Some(yard) = inst.yard_rect {
        rect_intersects(&target_rect, &yard)
    } else {
        false
    };

    // Caso 1: STORAGE (yard) → DISPATCH (fora do yard)
    if start_in_yard && !target_in_yard {
        // 1) só vertical até à linha Y do staging da dispatch
        if c.bl.y != target_bl.y {
            let dy = target_bl.y - c.bl.y;
            if dy > 0 {
                face_to(c, Direction::Up, cmds);
                move_forward(c, dy, cmds);
            } else {
                face_to(c, Direction::Down, cmds);
                move_forward(c, dy, cmds); // dy < 0 → anda para trás se necessário
            }
        }

        // 2) depois horizontal até à coluna X do staging
        if c.bl.x != target_bl.x {
            let dx = target_bl.x - c.bl.x;
            if dx > 0 {
                face_to(c, Direction::Right, cmds);
                move_forward(c, dx, cmds);
            } else {
                face_to(c, Direction::Left, cmds);
                move_forward(c, dx, cmds);
            }
        }

        // 3) orientar para a direção de staging (ex: Right na dispatch)
        face_to(c, target_dir, cmds);
        return;
    }

    // Caso 2: DISPATCH (fora) → STORAGE (yard)
    if !start_in_yard && target_in_yard {
        // 1) horizontal na linha da dispatch
        if c.bl.x != target_bl.x {
            let dx = target_bl.x - c.bl.x;
            if dx > 0 {
                face_to(c, Direction::Right, cmds);
                move_forward(c, dx, cmds);
            } else {
                face_to(c, Direction::Left, cmds);
                move_forward(c, dx, cmds);
            }
        }

        // 2) depois vertical para dentro do yard
        if c.bl.y != target_bl.y {
            let dy = target_bl.y - c.bl.y;
            if dy > 0 {
                face_to(c, Direction::Up, cmds);
                move_forward(c, dy, cmds);
            } else {
                face_to(c, Direction::Down, cmds);
                move_forward(c, dy, cmds);
            }
        }

        // 3) orientar para o staging_dir do storage (normalmente Up)
        face_to(c, target_dir, cmds);
        return;
    }

    // Caso 3: genérico (ambos no mesmo “tipo” de zona)
    // Manhattan simples: primeiro X, depois Y, e no fim face target_dir.
    if c.bl.x != target_bl.x {
        let dx = target_bl.x - c.bl.x;
        if dx > 0 {
            face_to(c, Direction::Right, cmds);
            move_forward(c, dx, cmds);
        } else {
            face_to(c, Direction::Left, cmds);
            move_forward(c, dx, cmds);
        }
    }

    if c.bl.y != target_bl.y {
        let dy = target_bl.y - c.bl.y;
        if dy > 0 {
            face_to(c, Direction::Up, cmds);
            move_forward(c, dy, cmds);
        } else {
            face_to(c, Direction::Down, cmds);
            move_forward(c, dy, cmds);
        }
    }

    face_to(c, target_dir, cmds);
}
