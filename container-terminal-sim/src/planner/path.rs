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

fn intersects_any_storage(inst: &Instance, bl: Point, dir: Direction) -> bool {
    let r = carrier_rect(bl, dir);
    for s in &inst.storages {
        if rect_intersects(&r, &s.rect) {
            return true;
        }
    }
    false
}

fn is_vertical(d: Direction) -> bool {
    matches!(d, Direction::Up | Direction::Down)
}
fn is_horizontal(d: Direction) -> bool {
    matches!(d, Direction::Left | Direction::Right)
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

fn face_to(inst: &Instance, c: &mut CarrierState, new_dir: Direction, cmds: &mut Vec<Command>) {
    if c.dir == new_dir { return; }

    // RULE: never rotate while in yard. Exit yard first.
    // This matches: inside yard -> only move / move -k, no faces.
    if in_yard(inst, c.bl, c.dir) {
        if let Some(yard) = inst.yard_rect {
            // pick closest exit vertically (works with vertical dir, and if horizontal appears,
            // move_along_y will rotate outside-yard only after we exit)
            let up_y = yard.y2 + 1;
            let down_y = yard.y1 - LONG;
            let dist_up = (c.bl.y - up_y).abs();
            let dist_down = (c.bl.y - down_y).abs();
            let exit_y = if dist_up <= dist_down { up_y } else { down_y };
            move_along_y(inst, c, exit_y, cmds);
        }
    }

    // Also: checker can reject rotation if we intersect a storage footprint.
    // So if we still intersect storages, step out first (outside yard).
    if intersects_any_storage(inst, c.bl, c.dir) {
        let cur = carrier_rect(c.bl, c.dir);
        let mut max_y2 = i32::MIN;
        let mut min_y1 = i32::MAX;
        let mut max_x2 = i32::MIN;
        let mut min_x1 = i32::MAX;
        let mut any = false;

        for s in &inst.storages {
            if rect_intersects(&cur, &s.rect) {
                any = true;
                max_y2 = max_y2.max(s.rect.y2);
                min_y1 = min_y1.min(s.rect.y1);
                max_x2 = max_x2.max(s.rect.x2);
                min_x1 = min_x1.min(s.rect.x1);
            }
        }

        if any {
            // Prefer moving along Y for vertical, along X for horizontal.
            if is_vertical(c.dir) {
                let up_y = max_y2 + 1;
                let down_y = min_y1 - LONG;
                let dist_up = (c.bl.y - up_y).abs();
                let dist_down = (c.bl.y - down_y).abs();
                let exit_y = if dist_up <= dist_down { up_y } else { down_y };
                move_along_y(inst, c, exit_y, cmds);
            } else {
                let left_x = min_x1 - LONG;
                let right_x = max_x2 + 1;
                let dist_left = (c.bl.x - left_x).abs();
                let dist_right = (c.bl.x - right_x).abs();
                let exit_x = if dist_left <= dist_right { left_x } else { right_x };
                move_along_x(inst, c, exit_x, cmds);
            }
        }
    }

    // Now rotation is safe.
    let t = c.time;
    cmds.push(Command::Face { t, dir: new_dir });
    c.time += 1;

    // Keep center fixed when changing footprint.
    let center = center_from_bl(c.bl, c.dir);
    c.dir = new_dir;
    c.bl  = bl_from_center(center, c.dir);
}


// move em frente ou marcha-atrás, consoante o sinal de `steps`
// duração = |steps|
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

fn move_along_y(inst: &Instance, c: &mut CarrierState, target_y: i32, cmds: &mut Vec<Command>) {
    let dy = target_y - c.bl.y;
    if dy == 0 { return; }

    let desired_dir = if dy > 0 { Direction::Up } else { Direction::Down };
    let steps = dy.abs();

    match c.dir {
        d if d == desired_dir => {
            move_forward(c, steps, cmds);
        }
        d if d == opposite(desired_dir) => {
            // marcha-atrás vertical (sem face)
            move_forward(c, -(steps), cmds);
        }
        _ => {
            // está horizontal → precisa de rodar para um vertical
            face_to(inst, c, desired_dir, cmds);
            move_forward(c, steps, cmds);
        }
    }
}

fn move_along_x(inst: &Instance, c: &mut CarrierState, target_x: i32, cmds: &mut Vec<Command>) {
    let dx = target_x - c.bl.x;
    if dx == 0 { return; }

    let desired_dir = if dx > 0 { Direction::Right } else { Direction::Left };
    let steps = dx.abs();

    match c.dir {
        d if d == desired_dir => {
            move_forward(c, steps, cmds);
        }
        d if d == opposite(desired_dir) => {
            // marcha-atrás horizontal (sem face)
            move_forward(c, -(steps), cmds);
        }
        _ => {
            // está vertical → precisa rodar para horizontal
            face_to(inst, c, desired_dir, cmds);
            move_forward(c, steps, cmds);
        }
    }
}

fn go_storage_to_dispatch(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let desired_center = center_from_bl(target_bl, target_dir);
    let bl_before = bl_from_center(desired_center, c.dir);

    // 1) ainda vertical: alinhar Y para o BL que preserva o centro
    move_along_y(inst, c, bl_before.y, cmds);

    // 2) se vamos ficar horizontais e ainda estamos no yard, sair antes
    if is_horizontal(target_dir) && in_yard(inst, c.bl, c.dir) {
        if let Some(yard) = inst.yard_rect {
            let up_y = yard.y2 + 1;
            let down_y = yard.y1 - LONG;
            let exit_y = if (c.bl.y - up_y).abs() <= (c.bl.y - down_y).abs() { up_y } else { down_y };
            move_along_y(inst, c, exit_y, cmds);
        }
    }

    // 3) rodar
    face_to(inst, c, target_dir, cmds);

    // 4) alinhar para o staging EXATO (agora já com footprint final)
    move_along_x(inst, c, target_bl.x, cmds);
    move_along_y(inst, c, target_bl.y, cmds);
}


fn go_dispatch_to_storage(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let desired_center = center_from_bl(target_bl, target_dir);
    let bl_before = bl_from_center(desired_center, c.dir);

    // 1) alinhar X na orientação atual
    move_along_x(inst, c, bl_before.x, cmds);

    // 2) se vamos ficar horizontais e estamos no yard, sair antes (defensivo)
    if is_horizontal(target_dir) && in_yard(inst, c.bl, c.dir) {
        if let Some(yard) = inst.yard_rect {
            let up_y = yard.y2 + 1;
            let down_y = yard.y1 - LONG;
            let exit_y = if (c.bl.y - up_y).abs() <= (c.bl.y - down_y).abs() { up_y } else { down_y };
            move_along_y(inst, c, exit_y, cmds);
        }
    }

    // 3) rodar
    face_to(inst, c, target_dir, cmds);

    // 4) alinhar staging EXATO na orientação final
    move_along_x(inst, c, target_bl.x, cmds);
    move_along_y(inst, c, target_bl.y, cmds);
}


pub fn go_to_pose(
    inst: &Instance,
    c: &mut CarrierState,
    target_bl: Point,
    target_dir: Direction,
    cmds: &mut Vec<Command>,
) {
    let yard_opt = inst.yard_rect;

    // centro desejado na pose final
    let desired_center = center_from_bl(target_bl, target_dir);

    // BL equivalente na orientação atual (para depois rodar sem deslocar staging)
    let bl_before = bl_from_center(desired_center, c.dir);

    if yard_opt.is_none() {
        move_along_x(inst, c, bl_before.x, cmds);
        move_along_y(inst, c, bl_before.y, cmds);
        face_to(inst, c, target_dir, cmds);

        // garantir staging exato
        move_along_x(inst, c, target_bl.x, cmds);
        move_along_y(inst, c, target_bl.y, cmds);
        return;
    }

    let start_in_yard  = in_yard(inst, c.bl, c.dir);
    let target_in_yard = in_yard(inst, target_bl, target_dir);

    if start_in_yard && !target_in_yard {
        go_storage_to_dispatch(inst, c, target_bl, target_dir, cmds);
        return;
    }

    if !start_in_yard && target_in_yard {
        go_dispatch_to_storage(inst, c, target_bl, target_dir, cmds);
        return;
    }

    // Caso genérico: alinhar para bl_before, rodar, e só no fim target_bl exato
    move_along_x(inst, c, bl_before.x, cmds);
    move_along_y(inst, c, bl_before.y, cmds);

    // se estamos no yard e precisamos ficar horizontais, sair antes (face_to também trata, mas deixo defensivo)
    if in_yard(inst, c.bl, c.dir) && is_horizontal(target_dir) {
        if let Some(yard) = yard_opt {
            let up_y = yard.y2 + 1;
            let down_y = yard.y1 - LONG;
            let exit_y = if (c.bl.y - up_y).abs() <= (c.bl.y - down_y).abs() { up_y } else { down_y };
            move_along_y(inst, c, exit_y, cmds);
        }
    }

    face_to(inst, c, target_dir, cmds);

    // staging exato (evita 100% “infeasible spot”)
    move_along_x(inst, c, target_bl.x, cmds);
    move_along_y(inst, c, target_bl.y, cmds);
}

