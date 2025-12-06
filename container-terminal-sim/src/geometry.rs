use crate::model::*;

pub fn enrich_instance_geometry(inst: &mut Instance) {
    compute_yard_rect(inst);
    compute_storage_staging(inst);
    compute_dispatch_staging(inst);
}

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

pub fn compute_storage_staging(inst: &mut Instance) {
    for s in &mut inst.storages {
        s.staging_bl = Some(Point {
            x: s.rect.x1 - 2,    // 2 à esquerda
            y: s.rect.y2,        // logo acima do storage
        });
        s.staging_dir = Some(Direction::Up);
    }
}



pub fn compute_dispatch_staging(inst: &mut Instance) {
    for d in &mut inst.dispatches {
        d.staging_bl = Some(Point {
            x: d.rect.x1 - 2,
            y: d.rect.y1 - 2,
        });
        d.staging_dir = Some(Direction::Right);
    }
}

