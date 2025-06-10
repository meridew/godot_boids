use glam::*;
use super::BoidProperties;

#[derive(Default)]
pub struct BoidData {
    pub positions_x: Vec<f32>,
    pub positions_y: Vec<f32>, 
    pub positions_z: Vec<f32>,
    pub velocities_x: Vec<f32>,
    pub velocities_y: Vec<f32>,
    pub velocities_z: Vec<f32>,
    pub max_speeds: Vec<f32>,
    pub max_forces: Vec<f32>,
    pub separations: Vec<f32>,
    pub alignments: Vec<f32>,
    pub cohesions: Vec<f32>,
    pub targetings: Vec<f32>,
    pub forces_x: Vec<f32>,
    pub forces_y: Vec<f32>,
    pub forces_z: Vec<f32>,
    pub count: usize,
}

impl BoidData {
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            positions_x: Vec::with_capacity(capacity),
            positions_y: Vec::with_capacity(capacity),
            positions_z: Vec::with_capacity(capacity),
            velocities_x: Vec::with_capacity(capacity),
            velocities_y: Vec::with_capacity(capacity),
            velocities_z: Vec::with_capacity(capacity),
            max_speeds: Vec::with_capacity(capacity),
            max_forces: Vec::with_capacity(capacity),
            separations: Vec::with_capacity(capacity),
            alignments: Vec::with_capacity(capacity),
            cohesions: Vec::with_capacity(capacity),
            targetings: Vec::with_capacity(capacity),
            forces_x: Vec::with_capacity(capacity),
            forces_y: Vec::with_capacity(capacity),
            forces_z: Vec::with_capacity(capacity),
            count: 0,
        }
    }
    
    pub fn add_boid(&mut self, pos: Vec3, vel: Vec3, props: &BoidProperties) {
        self.positions_x.push(pos.x);
        self.positions_y.push(pos.y);
        self.positions_z.push(pos.z);
        self.velocities_x.push(vel.x);
        self.velocities_y.push(vel.y);
        self.velocities_z.push(vel.z);
        self.max_speeds.push(props.max_speed);
        self.max_forces.push(props.max_force);
        self.separations.push(props.seperation);
        self.alignments.push(props.alignment);
        self.cohesions.push(props.cohesion);
        self.targetings.push(props.targeting);
        self.forces_x.push(0.0);
        self.forces_y.push(0.0);
        self.forces_z.push(0.0);
        self.count += 1;
    }
    
    pub fn get_position(&self, idx: usize) -> Vec3 {
        Vec3::new(
            self.positions_x[idx],
            self.positions_y[idx], 
            self.positions_z[idx]
        )
    }
    
    pub fn get_velocity(&self, idx: usize) -> Vec3 {
        Vec3::new(
            self.velocities_x[idx],
            self.velocities_y[idx],
            self.velocities_z[idx]
        )
    }
    
    pub fn set_force(&mut self, idx: usize, force: Vec3) {
        self.forces_x[idx] = force.x;
        self.forces_y[idx] = force.y;
        self.forces_z[idx] = force.z;
    }
    
    pub fn get_force(&self, idx: usize) -> Vec3 {
        Vec3::new(
            self.forces_x[idx],
            self.forces_y[idx],
            self.forces_z[idx]
        )
    }
    
    pub fn clear(&mut self) {
        self.positions_x.clear();
        self.positions_y.clear();
        self.positions_z.clear();
        self.velocities_x.clear();
        self.velocities_y.clear();
        self.velocities_z.clear();
        self.max_speeds.clear();
        self.max_forces.clear();
        self.separations.clear();
        self.alignments.clear();
        self.cohesions.clear();
        self.targetings.clear();
        self.forces_x.clear();
        self.forces_y.clear();
        self.forces_z.clear();
        self.count = 0;
    }
}