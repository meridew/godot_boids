use glam::*;
use godot::prelude::*;
use crate::BoidProperties;

// Flock trait - kept minimal for performance
pub trait Flock {
    fn get_flock_properties(&self) -> &crate::FlockProperties;
    fn get_target_position(&self) -> Option<Vec3>;
    fn get_boids(&self) -> impl Iterator<Item = (&InstanceId, (Vec3, Vec3, BoidProperties))>;
    fn get_boids_posvel(&self) -> Vec<(Vec3, Vec3)>;
    fn is_boid_processing(&self) -> bool;
}