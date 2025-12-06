use crate::model::*;

#[derive(Clone)]
pub struct CarrierState {
    pub id: Id,
    pub bl: Point,
    pub dir: Direction,
    pub carrying: Option<Id>,
    pub time: i32,
}

pub struct WorldState {
    pub carriers: Vec<CarrierState>,
    pub storage_stacks: Vec<Vec<Id>>,
}

impl WorldState {
    pub fn from_instance(inst: &Instance) -> Self {
        let carriers = inst.carriers.iter().map(|c| CarrierState {
            id: c.id,
            bl: c.bl,
            dir: c.dir,
            carrying: c.carrying,
            time: 0,
        }).collect();
        Self {
            carriers,
            storage_stacks: inst.storage_stacks.clone(),
        }
    }
}
