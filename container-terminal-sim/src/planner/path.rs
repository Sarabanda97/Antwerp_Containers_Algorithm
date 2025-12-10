use crate::model::{Instance, Rect, Point, Direction};
use crate::state::CarrierState;

#[derive(Clone, Debug)]
pub enum Command {
    Move   { t: i32, k: i32 },
    Face   { t: i32, dir: Direction },
    Load   { t: i32 },
    Unload { t: i32 },
}



const SHORT: i32 = 4; // lado curto
const LONG:  i32 = 8; // lado comprido

fn dims(dir: Direction) -> (i32, i32) {
    match dir {
        Direction::Up | Direction::Down => (SHORT, LONG),   // 4×8 vertical
        Direction::Left | Direction::Right => (LONG, SHORT) // 8×4 horizontal
    }
}

fn center_from_bl(bl: Point, dir: Direction) -> Point {
    let (w, h) = dims(dir);
    Point {
        x: bl.x + w / 2,
        y: bl.y + h / 2,
    }
}

fn bl_from_center(center: Point, dir: Direction) -> Point {
    let (w, h) = dims(dir);
    Point {
        x: center.x - w / 2,
        y: center.y - h / 2,
    }
}

// rect do carrier dado (bl, dir) – inclusivo
fn carrier_rect(bl: Point, dir: Direction) -> Rect {
    let (w, h) = dims(dir);
    Rect {
        x1: bl.x,
        y1: bl.y,
        x2: bl.x + w - 1,
        y2: bl.y + h - 1,
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

// Pequena ajuda para detectar a direcção oposta (para marcha-atrás)
fn opposite(dir: Direction) -> Direction {
    match dir {
        Direction::Up    => Direction::Down,
        Direction::Down  => Direction::Up,
        Direction::Left  => Direction::Right,
        Direction::Right => Direction::Left,
    }
}



fn face_to(c: &mut CarrierState, new_dir: Direction, cmds: &mut Vec<Command>) {
    if c.dir == new_dir { return; }

    let t = c.time;
    cmds.push(Command::Face { t, dir: new_dir });
    c.time += 1;

    // rotação com centro fixo
    let center = center_from_bl(c.bl, c.dir);
    c.dir = new_dir;
    c.bl  = bl_from_center(center, c.dir);

       println!(
        "% DEBUG face @ t={} -> dir={:?}, bl=({}, {})",
        t, c.dir, c.bl.x, c.bl.y
    );

}

/// move em frente ou marcha-atrás, consoante o sinal de `steps`
/// duração = |steps|
fn move_forward(c: &mut CarrierState, steps: i32, cmds: &mut Vec<Command>) {
    if steps == 0 { return; }
    let t_start = c.time;
    let t_end   = t_start + steps.abs();
    
    cmds.push(Command::Move { t: t_start, k: steps });
    c.time = t_end;

    let (dx, dy) = match c.dir {
        Direction::Up    => (0,  1),
        Direction::Down  => (0, -1),
        Direction::Left  => (-1, 0),
        Direction::Right => (1,  0),
    };

    c.bl.x += dx * steps;
    c.bl.y += dy * steps;

    println!(
        "% DEBUG move @ t={}..{} k={} dir={:?} -> bl=({}, {})",
        t_start, t_end, steps, c.dir, c.bl.x, c.bl.y
    );

}



fn move_along_y(c: &mut CarrierState, target_y: i32, cmds: &mut Vec<Command>) {
    let dy = target_y - c.bl.y;
    if dy == 0 { return; }

    let desired_dir = if dy > 0 { Direction::Up } else { Direction::Down };
    let steps = dy.abs();

    match c.dir {
        d if d == desired_dir => {
            // já virado para o lado certo → forward
            move_forward(c, steps, cmds);
        }
        d if d == opposite(desired_dir) => {
            // virado ao contrário → marcha-atrás
            move_forward(c, -(steps), cmds);
        }
        _ => {
            // horizontal → roda para a direcção desejada e anda para a frente
            face_to(c, desired_dir, cmds);
            move_forward(c, steps, cmds);
        }
    }
}

fn move_along_x(c: &mut CarrierState, target_x: i32, cmds: &mut Vec<Command>) {
    let dx = target_x - c.bl.x;
    if dx == 0 { return; }

    let desired_dir = if dx > 0 { Direction::Right } else { Direction::Left };
    let steps = dx.abs();

    match c.dir {
        d if d == desired_dir => {
            move_forward(c, steps, cmds);
        }
        d if d == opposite(desired_dir) => {
            move_forward(c, -(steps), cmds);
        }
        _ => {
            face_to(c, desired_dir, cmds);
            move_forward(c, steps, cmds);
        }
    }
}



fn go_storage_to_dispatch(
    _inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    // 1) Storage (no yard) → alinhar Y com a dispatch (sempre vertical)
    move_along_y(c, target_bl.y, cmds);

    // 2) já fora do yard → alinhar X com o staging da dispatch
    move_along_x(c, target_bl.x, cmds);

    // 3) garantir direção final (Right normalmente)
    face_to(c, target_dir, cmds);
}


fn go_dispatch_to_storage(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let yard = inst.yard_rect.expect("yard_rect esperado");

    // 1) Despacho está cá em baixo (y ≈ 5), fora do yard.
    //    Primeiro alinhar X com o staging da storage enquanto ainda estamos abaixo do yard:
    move_along_x(c, target_bl.x, cmds); // horizontal FORA do yard

    // 2) Agora subir em Y até à staging do storage (entrando no yard só em vertical):
    move_along_y(c, target_bl.y, cmds); // só vertical dentro do yard

    // 3) Garantir orientação final (Up)
    face_to(c, target_dir, cmds);
}


pub fn go_to_pose(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let yard_opt = inst.yard_rect;

    // se por algum motivo não houver yard_rect, faz Manhattan simples
    if yard_opt.is_none() {
        move_along_x(c, target_bl.x, cmds);
        move_along_y(c, target_bl.y, cmds);
        face_to(c, target_dir, cmds);
        return;
    }

    let start_in_yard  = in_yard(inst, c.bl, c.dir);
    let target_in_yard = in_yard(inst, target_bl, target_dir);

    // 1) STORAGE (dentro do yard) → DISPATCH (fora)
    if start_in_yard && !target_in_yard {
        go_storage_to_dispatch(inst, c, target_bl, target_dir, cmds);
        return;
    }

    // 2) DISPATCH (fora) → STORAGE (dentro do yard)
    if !start_in_yard && target_in_yard {
        go_dispatch_to_storage(inst, c, target_bl, target_dir, cmds);
        return;
    }

    // 3) Caso genérico (ambos fora, ou ambos dentro sem regra especial):
    move_along_x(c, target_bl.x, cmds);
    move_along_y(c, target_bl.y, cmds);
    face_to(c, target_dir, cmds);
}
