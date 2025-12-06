pub type Id = i32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Point { pub x: i32, pub y: i32 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect { pub x1: i32, pub y1: i32, pub x2: i32, pub y2: i32 }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction { Up, Down, Left, Right }

#[derive(Clone, Debug)]
pub struct Crane {
    pub id: Id,
    pub rect: Rect,
    pub dispatch_ids: Vec<Id>,
}

#[derive(Clone, Debug)]
pub struct Dispatch {
    pub id: Id,
    pub crane_id: Id,
    pub rect: Rect,          // 4×2, inclusive (x2 = x1+3, y2 = y1+1)
    pub staging_bl: Option<Point>,
    pub staging_dir: Option<Direction>,
}

#[derive(Clone, Debug)]
pub struct Storage {
    pub id: Id,
    pub rect: Rect,          // 2×4, inclusive (x2 = x1+1, y2 = y1+3)
    pub staging_bl: Option<Point>,
    pub staging_dir: Option<Direction>,
}

#[derive(Clone, Debug)]
pub struct Carrier {
    pub id: Id,
    pub assigned_crane: Id,
    pub bl: Point,           // bottom-left
    pub dir: Direction,
    pub carrying: Option<Id>,
    pub size: (i32, i32),    // (4,8)
}

#[derive(Clone, Debug)]
pub enum Demand {
    Unload { dispatch_id: Id, container_id: Id, storage_id: Id },
    Load   { dispatch_id: Id, container_id: Id },
}

#[derive(Clone, Debug)]
pub struct ShipBlock {
    pub ship_id: Id,
    pub crane_id: Option<Id>,
    pub operations: Vec<Demand>,
}

#[derive(Clone, Debug)]
pub struct Instance {
    pub width: i32,
    pub height: i32,
    pub cranes: Vec<Crane>,
    pub dispatches: Vec<Dispatch>,
    pub storages: Vec<Storage>,
    pub carriers: Vec<Carrier>,
    pub storage_stacks: Vec<Vec<Id>>,
    pub demands: Vec<Demand>,
    pub total_new_containers: Option<i32>,
    pub ships: Vec<ShipBlock>,

    // enriquecido pela geometry.rs
    pub yard_rect: Option<Rect>,
}
