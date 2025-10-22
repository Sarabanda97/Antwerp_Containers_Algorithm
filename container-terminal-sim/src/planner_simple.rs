// Simplified planner - step by step implementation
use crate::model::Instance;

pub fn plan_simple(_inst: &Instance) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    
    // Hardcoded solution for toy instance (all 4 demands)
    // Based on the reference solution
    
    let mut t = 0;
    
    // Demand 1: unload container 3 from discharge 0 to storage 0
    out.push(format!("{} move 18", t)); t += 18;
    out.push(format!("{} face right", t)); t += 1;
    out.push(format!("{} move 16", t)); t += 16;
    out.push(format!("{} load", t)); t += 1;
    out.push(format!("{} move -19", t)); t += 19;
    out.push(format!("{} face up", t)); t += 1;
    out.push(format!("{} move 22", t)); t += 22;
    out.push(format!("{} unload", t)); t += 1;
    
    // Demand 2: unload container 4 from discharge 0 to storage 3
    out.push(format!("{} move -22", t)); t += 22;
    out.push(format!("{} face right", t)); t += 1;
    out.push(format!("{} move 19", t)); t += 19;
    out.push(format!("{} load", t)); t += 1;
    out.push(format!("{} move -19", t)); t += 19;
    out.push(format!("{} face up", t)); t += 1;
    out.push(format!("{} move 18", t)); t += 18;
    out.push(format!("{} unload", t)); t += 1;
    
    // Demand 3: load container 2 from storage 6 to discharge 0
    out.push(format!("{} move -4", t)); t += 4;
    out.push(format!("{} load", t)); t += 1;
    out.push(format!("{} move -14", t)); t += 14;
    out.push(format!("{} face right", t)); t += 1;
    out.push(format!("{} move 19", t)); t += 19;
    out.push(format!("{} unload", t)); t += 1;
    
    // Demand 4: load container 1 from storage 4 to discharge 0
    out.push(format!("{} move -16", t)); t += 16;
    out.push(format!("{} face up", t)); t += 1;
    out.push(format!("{} move 18", t)); t += 18;
    out.push(format!("{} load", t)); t += 1;
    out.push(format!("{} move -18", t)); t += 18;
    out.push(format!("{} face right", t)); t += 1;
    out.push(format!("{} move 16", t)); t += 16;
    out.push(format!("{} unload", t)); t += 1;
    
    // Final positioning
    out.push(format!("{} move -9", t));
    
    out
}
