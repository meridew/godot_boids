use glam::*;
use godot::prelude::*;

pub mod types_2d;
pub mod types_3d;
pub mod properties;

pub use types_2d::*;
pub use types_3d::*;
pub use properties::*;

// Core boid trait for Godot integration
pub trait Boid {
    fn apply_force(&mut self, force: Vec3);
    fn get_boid_position(&self) -> Vec3;
    fn get_boid_velocity(&self) -> Vec3;
    fn get_boid_properties(&self) -> &BoidProperties;
    fn get_flock_id(&self) -> InstanceId;
}