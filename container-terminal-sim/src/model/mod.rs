use std::fmt;

#[derive(Debug, Clone)]
pub struct Crane {
    pub id: i32,
    pub rect: (i32,i32,i32,i32),           // x1,y1,x2,y2
    pub dispatch_positions: Vec<(i32,i32)>,// bottom-lefts das dispatch (4x2)
}

#[derive(Debug, Clone)]
pub struct Storage {
    pub id: i32,
    pub bl: (i32,i32), // bottom-left (2x4)
}

#[derive(Debug, Clone)]
pub struct Carrier {
    pub id: i32,
    pub crane_id: i32,
    pub bl: (i32,i32), // bottom-left (4x8)
    // future: dir, carrying
}

#[derive(Debug, Clone)]
pub enum Demand {
    Load   { crane_id: i32, container_id: i32 },
    Unload { crane_id: i32, container_id: i32, storage_id: i32 },
}

#[derive(Debug, Clone)]
pub struct Instance {
    pub width: i32,
    pub height: i32,
    pub cranes: Vec<Crane>,
    pub storages: Vec<Storage>,
    pub carriers: Vec<Carrier>,
    pub containers_init: Vec<(i32,i32)>, // (container_id, storage_id)
    pub demands: Vec<Demand>,
}

impl fmt::Display for Instance {
    fn fmt(&self, f:&mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Map: {} x {}", self.width, self.height)?;
        writeln!(f, "Cranes: {}", self.cranes.len())?;
        for c in &self.cranes {
            writeln!(f, "  Crane {} dispatches: {:?}", c.id, c.dispatch_positions)?;
        }
        writeln!(f, "Storages: {}", self.storages.len())?;
        writeln!(f, "Carriers: {}", self.carriers.len())?;
        writeln!(f, "Initial containers: {}", self.containers_init.len())?;
        writeln!(f, "Demands: {}", self.demands.len())
    }
}
