use std::fmt;

pub type Id = i32;
pub type Coord = i32;

/* ===================== Geometria ===================== */

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Point { pub x: Coord, pub y: Coord }

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Retângulo em coordenadas de grelha.
/// Convenção: inclusivo em ambos os extremos (x1..=x2, y1..=y2).
pub struct Rect { pub x1: Coord, pub y1: Coord, pub x2: Coord, pub y2: Coord }

impl Rect {
    pub fn width(&self) -> Coord  { self.x2 - self.x1 + 1 }
    pub fn height(&self) -> Coord { self.y2 - self.y1 + 1 }
    pub fn contains(&self, p: Point) -> bool {
        p.x >= self.x1 && p.x <= self.x2 && p.y >= self.y1 && p.y <= self.y2
    }
}

/* ===================== Direção / Dimensão ===================== */

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction { Up, Right, Down, Left }

/* ===================== Terminal ===================== */

#[derive(Debug, Clone)]
pub struct Crane {
    pub id: Id,
    pub rect: Rect,            // envelope da grua
    pub dispatch_ids: Vec<Id>, // liga aos Dispatch desta grua
}

#[derive(Debug, Clone)]
pub struct Dispatch {
    pub id: Id,                // "discharge/dispatch id" usado nas demands
    pub crane_id: Id,
    pub rect: Rect,            // 4×2 (a partir do (x,y) do ficheiro)
}

#[derive(Debug, Clone)]
pub struct Storage {
    pub id: Id,
    pub rect: Rect,            // 2×4 (a partir do BL do ficheiro)
}

#[derive(Debug, Clone)]
pub struct Carrier {
    pub id: Id,
    pub assigned_crane: Id,
    pub bl: Point,                 // bottom-left (4×8)
    pub dir: Direction,            // inicial: Down
    pub carrying: Option<Id>,      // None no início
    pub size: (Coord, Coord),      // (4, 8) – só para referência rápida
}

/* ===================== Operações ===================== */

#[derive(Debug, Clone)]
pub enum Demand {
    /// Retira do navio na secção dispatch_id e coloca na storage_id.
    Unload { dispatch_id: Id, container_id: Id, storage_id: Id },
    /// Leva o contentor até a secção dispatch_id para carregar no navio.
    Load   { dispatch_id: Id, container_id: Id },
}

/* ===================== Instância ===================== */

#[derive(Debug, Clone)]
pub struct Instance {
    // mapa
    pub width: Coord,
    pub height: Coord,

    // recursos fixos
    pub cranes: Vec<Crane>,
    pub dispatches: Vec[Dispatch],
    pub storages: Vec<Storage>,
    pub carriers: Vec<Carrier>,

    // estado inicial
    /// storage_id -> stack bottom..top (capacidade 2)
    pub storage_stacks: Vec<Vec<Id>>,

    // ordens operacionais
    pub demands: Vec<Demand>,
}

/* ===================== Debug amigável ===================== */

impl fmt::Display for Instance {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Map: {} x {}", self.width, self.height)?;
        writeln!(f, "Cranes: {}", self.cranes.len())?;
        for c in &self.cranes {
            writeln!(f, "  Crane {} rect=({},{},{},{}) dispatches={:?}",
                c.id, c.rect.x1, c.rect.y1, c.rect.x2, c.rect.y2, c.dispatch_ids)?;
        }
        writeln!(f, "Dispatches: {}", self.dispatches.len())?;
        writeln!(f, "Storages: {}", self.storages.len())?;
        writeln!(f, "Carriers: {}", self.carriers.len())?;
        writeln!(f, "Storage stacks:")?;
        for (sid, st) in self.storage_stacks.iter().enumerate() {
            writeln!(f, "  storage {}: {:?}", sid, st)?;
        }
        writeln!(f, "Demands: {}", self.demands.len())
    }
}
