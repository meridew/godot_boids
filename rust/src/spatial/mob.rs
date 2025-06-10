use glam::*;

pub mod ultra_hash;

pub use ultra_hash::*;

// Spatial data structure trait for different implementations
pub trait SpatialStructure {
    /// Rebuild the spatial structure with new positions
    fn rebuild(&mut self, positions: &[Vec3]);
    
    /// Query neighbors within radius of a position
    fn query_neighbors(&self, position: Vec3, radius: f32) -> Vec<u32>;
    
    /// Get the optimal cell size for this structure
    fn get_cell_size(&self) -> f32;
}