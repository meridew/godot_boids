use glam::*;
use rustc_hash::FxHashMap;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridCell {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

pub struct SpatialGrid {
    cell_size: f32,
    grid: FxHashMap<GridCell, Vec<usize>>, // boid indices
}

impl SpatialGrid {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            grid: FxHashMap::default(),
        }
    }

    fn get_cell(&self, pos: Vec3) -> GridCell {
        GridCell {
            x: (pos.x / self.cell_size).floor() as i32,
            y: (pos.y / self.cell_size).floor() as i32,
            z: (pos.z / self.cell_size).floor() as i32,
        }
    }

    pub fn clear(&mut self) {
        for bucket in self.grid.values_mut() {
            bucket.clear();
        }
    }

    pub fn insert(&mut self, boid_index: usize, position: Vec3) {
        let cell = self.get_cell(position);
        self.grid.entry(cell).or_default().push(boid_index);
    }

    pub fn get_neighbors(&self, position: Vec3, radius: f32) -> Vec<usize> {
        let center_cell = self.get_cell(position);
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        
        let mut neighbors = Vec::new();
        
        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                for dz in -cell_radius..=cell_radius {
                    let cell = GridCell {
                        x: center_cell.x + dx,
                        y: center_cell.y + dy,
                        z: center_cell.z + dz,
                    };
                    
                    if let Some(boids) = self.grid.get(&cell) {
                        neighbors.extend(boids.iter().copied());
                    }
                }
            }
        }
        
        neighbors
    }

    pub fn get_neighbors_2d(&self, position: Vec3, radius: f32) -> Vec<usize> {
        let center_cell = self.get_cell(position);
        let cell_radius = (radius / self.cell_size).ceil() as i32;
        
        let mut neighbors = Vec::new();
        
        // For 2D, only iterate over x and y
        for dx in -cell_radius..=cell_radius {
            for dy in -cell_radius..=cell_radius {
                let cell = GridCell {
                    x: center_cell.x + dx,
                    y: center_cell.y + dy,
                    z: center_cell.z, // Keep z the same for 2D
                };
                
                if let Some(boids) = self.grid.get(&cell) {
                    neighbors.extend(boids.iter().copied());
                }
            }
        }
        
        neighbors
    }
}