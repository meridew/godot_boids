use glam::*;
use crate::{BoidProperties, FlockProperties};

pub mod ultra;

pub use ultra::*;

// Core algorithm trait for extensibility
pub trait BoidAlgorithm {
    /// Process all boids and update their forces
    fn process_boids(&mut self, boids_data: &mut [BoidInstance], flock_props: &FlockProperties, target_pos: Option<Vec3>);
}

// Lightweight boid instance for algorithm processing
#[repr(C)]
#[derive(Clone)]
pub struct BoidInstance {
    pub position: Vec3,
    pub velocity: Vec3,
    pub properties: BoidProperties,
    pub force: Vec3,
}

impl BoidInstance {
    #[inline(always)]
    pub fn new(position: Vec3, velocity: Vec3, properties: BoidProperties) -> Self {
        Self {
            position,
            velocity,
            properties,
            force: Vec3::ZERO,
        }
    }
}