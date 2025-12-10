use crate::model::{Instance, Rect, Point, Direction};

pub fn enrich_instance_geometry(inst: &mut Instance) {
    compute_yard_rect(inst);
    compute_storage_staging(inst);
    compute_dispatch_staging(inst);
}

// Yard = bounding box que cobre todos os storages
pub fn compute_yard_rect(inst: &mut Instance) {
    if inst.storages.is_empty() {
        inst.yard_rect = None;
        return;
    }

    let mut x1 = i32::MAX;
    let mut y1 = i32::MAX;
    let mut x2 = i32::MIN;
    let mut y2 = i32::MIN;

    for s in &inst.storages {
        x1 = x1.min(s.rect.x1);
        y1 = y1.min(s.rect.y1);
        x2 = x2.max(s.rect.x2);
        y2 = y2.max(s.rect.y2);
    }

    inst.yard_rect = Some(Rect { x1, y1, x2, y2 });
}

// Staging vertical para cada storage:
// carrier 4×8 (vertical) a "estradear" o rect 2×4.
pub fn compute_storage_staging(inst: &mut Instance) {
    for s in &mut inst.storages {
        // storage rect = 2×4: [x1..x1+1] × [y1..y1+3]
        //
        // Queremos o carrier 4×8 (Up) centrado no storage:
        //  - em X: estende 1 célula para cada lado → bl.x = x1 - 1
        //  - em Y: estende 2 para baixo e 2 para cima → bl.y = y1 - 2
        s.staging_bl = Some(Point {
            x: s.rect.x1 - 1,
            y: s.rect.y1 - 2,
        });
        s.staging_dir = Some(Direction::Up);
    }
}

// Staging horizontal para cada dispatch:
// carrier 8×4 (horizontal) centrado no rect 4×2.
pub fn compute_dispatch_staging(inst: &mut Instance) {
    for d in &mut inst.dispatches {
        // dispatch rect = 4×2: [x1..x1+3] × [y1..y1+1]
        //
        // Carrier 8×4 (Right) centrado:
        //  - em X: 2 células para cada lado → bl.x = x1 - 2
        //  - em Y: 1 para baixo e 1 para cima → bl.y = y1 - 1
        d.staging_bl = Some(Point {
            x: d.rect.x1 - 2,
            y: d.rect.y1 - 1,
        });
        d.staging_dir = Some(Direction::Right); // carrier horizontal
    }
}

