use crate::model::*;
use crate::state::CarrierState;

#[derive(Clone, Debug)]
pub enum Command {
    Move  { t: i32, k: i32 },
    Face  { t: i32, dir: Direction },
    Load  { t: i32 },
    Unload{ t: i32 },
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
    // simplificação: assumimos que a bottom-left não muda na rotação
}

/// move em frente ou marcha-atrás consoante o sinal de `steps`
/// tempo soma |steps|
fn move_forward(c: &mut CarrierState, steps: i32, cmds: &mut Vec<Command>) {
    if steps == 0 { return; }
    let t = c.time;
    cmds.push(Command::Move { t, k: steps });
    c.time += steps.abs();

    match c.dir {
        Direction::Up    => { c.bl.y += steps; }
        Direction::Down  => { c.bl.y -= steps; }
        Direction::Left  => { c.bl.x -= steps; }
        Direction::Right => { c.bl.x += steps; }
    }
}

// helpers Manhattan que já usam marcha-atrás quando dá jeito
fn move_along_y(c: &mut CarrierState, target_y: i32, cmds: &mut Vec<Command>) {
    let dy = target_y - c.bl.y;
    if dy == 0 { return; }

    match c.dir {
        Direction::Up | Direction::Down => {
            // já estamos verticais – usamos forward/backward
            move_forward(c, dy, cmds);
        }
        Direction::Left | Direction::Right => {
            // estamos horizontais → temos de rodar
            if dy > 0 {
                face_to(c, Direction::Up, cmds);
            } else {
                face_to(c, Direction::Down, cmds);
            }
            move_forward(c, dy, cmds);
        }
    }
}

fn move_along_x(c: &mut CarrierState, target_x: i32, cmds: &mut Vec<Command>) {
    let dx = target_x - c.bl.x;
    if dx == 0 { return; }

    match c.dir {
        Direction::Left | Direction::Right => {
            // já estamos horizontais – usamos forward/backward
            move_forward(c, dx, cmds);
        }
        Direction::Up | Direction::Down => {
            // estamos verticais → temos de rodar
            if dx > 0 {
                face_to(c, Direction::Right, cmds);
            } else {
                face_to(c, Direction::Left, cmds);
            }
            move_forward(c, dx, cmds);
        }
    }
}

pub fn go_to_pose(inst: &Instance,
                  c: &mut CarrierState,
                  target_bl: Point,
                  target_dir: Direction,
                  cmds: &mut Vec<Command>) 
{
    let yard = inst.yard_rect.unwrap();
    let start_in_yard  = rect_intersects(&carrier_rect(c.bl, c.dir), &yard);
    let target_in_yard = rect_intersects(&carrier_rect(target_bl, target_dir), &yard);

    if start_in_yard && !target_in_yard {
        go_storage_to_dispatch(inst, c, target_bl, target_dir, cmds);
        return;
    }

    if !start_in_yard && target_in_yard {
        go_dispatch_to_storage(inst, c, target_bl, target_dir, cmds);
        return;
    }

    // fallback: shouldn't happen in toy
    face_to(c, Direction::Right, cmds);
    move_forward(c, target_bl.x - c.bl.x, cmds);
    face_to(c, Direction::Up, cmds);
    move_forward(c, target_bl.y - c.bl.y, cmds);
    face_to(c, target_dir, cmds);
}
pub fn go_storage_to_dispatch(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let yard = inst.yard_rect.unwrap();

    // 1) sair do yard — sempre vertical
    face_to(c, Direction::Down, cmds);
    let exit_y = yard.y1 - 1;
    move_forward(c, c.bl.y - exit_y, cmds);

    // 2) alinhar verticalmente com o staging da dispatch
    let dy = target_bl.y - c.bl.y;
    if dy > 0 { face_to(c, Direction::Up, cmds); move_forward(c, dy, cmds); }
    else      { face_to(c, Direction::Down, cmds); move_forward(c, -dy, cmds); }

    // 3) alinhar horizontalmente (sempre para a direita)
    let dx = target_bl.x - c.bl.x;
    face_to(c, Direction::Right, cmds);
    move_forward(c, dx, cmds);

    // 4) pose final
    face_to(c, target_dir, cmds);
}
pub fn go_dispatch_to_storage(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let yard = inst.yard_rect.unwrap();

    // 1) mover para a esquerda até ficar alinhado com a coluna do yard
    face_to(c, Direction::Left, cmds);
    let dx = c.bl.x - (yard.x1 - 5);
    move_forward(c, dx, cmds);

    // 2) alinhar verticalmente com a storage
    let dy = target_bl.y - c.bl.y;
    if dy > 0 { face_to(c, Direction::Up, cmds); move_forward(c, dy, cmds); }
    else      { face_to(c, Direction::Down, cmds); move_forward(c, -dy, cmds); }

    // 3) alinhar horizontalmente ao staging
    let dx2 = target_bl.x - c.bl.x;
    if dx2 > 0 { face_to(c, Direction::Right, cmds); move_forward(c, dx2, cmds); }
    else       { face_to(c, Direction::Left, cmds); move_forward(c, -dx2, cmds); }

    // 4) pose final
    face_to(c, target_dir, cmds);
}



