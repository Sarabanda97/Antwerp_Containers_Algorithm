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
    // **IMPORTANTE**: para simplificar, assumimos que a bottom-left NÃO muda com o face.
    // O checker é que aplica a geometria real; nós só usamos isto para ter um planeador consistente.
}

fn move_forward(c: &mut CarrierState, steps: i32, cmds: &mut Vec<Command>) {
    if steps == 0 { return; }
    let t = c.time;
    cmds.push(Command::Move { t, k: steps });
    c.time += steps;

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
    // Estratégia:
    // 1) Se estamos no yard e precisamos mexer em X ou ficar horizontais,
    //    primeiro saímos do yard só com movimentos verticais.
    // 2) Fora do yard: alinhamos X (horizontalmente).
    // 3) Depois alinhamos Y (verticalmente, eventualmente entrando no yard).
    // 4) Fazemos face para a direção final.
    //
    // Ignoramos colisões e obstáculos (exceto o yard como regra de movimento).

    // 1) Sair do yard se necessário
    let mut need_horizontal = c.bl.x != target_bl.x;
    let mut need_horizontal_dir = matches!(target_dir, Direction::Left | Direction::Right);

    if in_yard(inst, c.bl, c.dir) && (need_horizontal || need_horizontal_dir) {
        // Queremos ir na direção vertical aproximando-nos do target em Y
        let going_down = target_bl.y < c.bl.y;
        if going_down {
            face_to(c, Direction::Down, cmds);
        } else {
            face_to(c, Direction::Up, cmds);
        }

        // Anda passo a passo até sair do yard
        while in_yard(inst, c.bl, c.dir) {
            move_forward(c, 1, cmds);
        }
    }

    // 2) Alinhar X (horizontal) - fora do yard
    if c.bl.x != target_bl.x {
        let dx = target_bl.x - c.bl.x;
        if dx > 0 {
            face_to(c, Direction::Right, cmds);
            move_forward(c, dx, cmds);
        } else {
            face_to(c, Direction::Left, cmds);
            move_forward(c, -dx, cmds);
        }
    }

    // 3) Alinhar Y (vertical)
    if c.bl.y != target_bl.y {
        let dy = target_bl.y - c.bl.y;
        if dy > 0 {
            face_to(c, Direction::Up, cmds);
            move_forward(c, dy, cmds);
        } else {
            face_to(c, Direction::Down, cmds);
            move_forward(c, -dy, cmds);
        }
    }

    // 4) Orientar para a direção final
    // Aqui a regra do yard já está respeitada:
    // - staging de storage é vertical (Up) e está dentro do yard;
    // - staging de dispatch é horizontal (Right) e está fora do yard.
    face_to(c, target_dir, cmds);
}
