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
        let sx = s.rect.x1;
        let sy = s.rect.y1;

        // carrier 4x8 em pé, storage 2x4 “no meio”
        let mut bl = Point { x: sx - 1, y: sy - 2 };
        let dir = Direction::Up;

        if bl.x < 0 { bl.x = 0; }
        if bl.y < 0 { bl.y = 0; }
        if bl.x + 3 > inst.width { bl.x = inst.width - 3; }
        if bl.y + 7 > inst.height { bl.y = inst.height - 7; }

        s.staging_bl = Some(bl);
        s.staging_dir = Some(dir);
    }
}

pub fn compute_dispatch_staging(inst: &mut Instance) {
    for d in &mut inst.dispatches {
        let dx = d.rect.x1;
        let dy = d.rect.y1;

        // carrier 8x4 deitado, dispatch 4x2 “lá dentro”
        let mut bl = Point { x: dx - 2, y: dy - 1 };
        let dir = Direction::Right;

        if bl.x < 0 { bl.x = 0; }
        if bl.y < 0 { bl.y = 0; }
        if bl.x + 7 > inst.width { bl.x = inst.width - 7; }
        if bl.y + 3 > inst.height { bl.y = inst.height - 3; }

        d.staging_bl = Some(bl);
        d.staging_dir = Some(dir);
    }
}
