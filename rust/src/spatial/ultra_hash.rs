use glam::*;
use rustc_hash::FxHashMap;
use super::SpatialStructure;

// Forward declaration to avoid circular dependency
// We'll implement this method directly rather than through the trait
pub struct UltraSpatialHash {
    cell_size: f32,
    inv_cell_size: f32,
    buckets: FxHashMap<u64, Vec<u32>>,
    bucket_pool: Vec<Vec<u32>>, // Reuse vectors to eliminate allocations
}

impl UltraSpatialHash {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            inv_cell_size: 1.0 / cell_size,
            buckets: FxHashMap::default(),
            bucket_pool: Vec::with_capacity(2000), // Pre-allocate bucket pool
        }
    }
    
    #[inline(always)]
    fn hash_position(&self, pos: Vec3) -> u64 {
        // Fast Morton encoding for spatial locality
        let x = (pos.x * self.inv_cell_size) as i32;
        let y = (pos.y * self.inv_cell_size) as i32;
        let z = (pos.z * self.inv_cell_size) as i32;
        
        // 21-bit Morton encoding (supports large worlds)
        (((x as u64) & 0x1fffff) << 42) | 
        (((y as u64) & 0x1fffff) << 21) | 
        ((z as u64) & 0x1fffff)
    }
    
    pub fn rebuild_from_processor(&mut self, processor: &crate::algorithms::UltraBoidProcessor) {
        // Return all buckets to pool for reuse
        for (_, mut bucket) in self.buckets.drain() {
            bucket.clear();
            if bucket.capacity() >= 8 && bucket.capacity() <= 64 {
                self.bucket_pool.push(bucket);
            }
        }
        
        // Populate spatial hash
        for i in 0..processor.count {
            let pos = processor.get_position(i);
            let hash = self.hash_position(pos);
            
            let bucket = self.buckets.entry(hash).or_insert_with(|| {
                self.bucket_pool.pop().unwrap_or_else(|| Vec::with_capacity(16))
            });
            bucket.push(i as u32);
        }
    }
    
    #[inline(always)]
    pub fn query_neighbors(&self, pos: Vec3, radius: f32) -> Vec<u32> {
        let mut neighbors = Vec::with_capacity(128); // Pre-size for typical neighborhood
        let grid_radius = (radius * self.inv_cell_size).ceil() as i32;
        
        let center_x = (pos.x * self.inv_cell_size) as i32;
        let center_y = (pos.y * self.inv_cell_size) as i32;
        let center_z = (pos.z * self.inv_cell_size) as i32;
        
        // Iterate through neighboring cells
        for dx in -grid_radius..=grid_radius {
            for dy in -grid_radius..=grid_radius {
                for dz in -grid_radius..=grid_radius {
                    let x = center_x + dx;
                    let y = center_y + dy;
                    let z = center_z + dz;
                    
                    let hash = (((x as u64) & 0x1fffff) << 42) | 
                              (((y as u64) & 0x1fffff) << 21) | 
                              ((z as u64) & 0x1fffff);
                    
                    if let Some(bucket) = self.buckets.get(&hash) {
                        neighbors.extend_from_slice(bucket);
                    }
                }
            }
        }
        
        neighbors
    }
    
    #[inline(always)]
    pub fn query_neighbors_2d(&self, pos: Vec3, radius: f32) -> Vec<u32> {
        let mut neighbors = Vec::with_capacity(64); // Smaller for 2D
        let grid_radius = (radius * self.inv_cell_size).ceil() as i32;
        
        let center_x = (pos.x * self.inv_cell_size) as i32;
        let center_y = (pos.y * self.inv_cell_size) as i32;
        let center_z = (pos.z * self.inv_cell_size) as i32;
        
        // Only iterate over x and y for 2D
        for dx in -grid_radius..=grid_radius {
            for dy in -grid_radius..=grid_radius {
                let x = center_x + dx;
                let y = center_y + dy;
                let z = center_z; // Fixed Z for 2D
                
                let hash = (((x as u64) & 0x1fffff) << 42) | 
                          (((y as u64) & 0x1fffff) << 21) | 
                          ((z as u64) & 0x1fffff);
                
                if let Some(bucket) = self.buckets.get(&hash) {
                    neighbors.extend_from_slice(bucket);
                }
            }
        }
        
        neighbors
    }
}

impl SpatialStructure for UltraSpatialHash {
    fn rebuild(&mut self, positions: &[Vec3]) {
        // Return buckets to pool
        for (_, mut bucket) in self.buckets.drain() {
            bucket.clear();
            if bucket.capacity() >= 8 && bucket.capacity() <= 64 {
                self.bucket_pool.push(bucket);
            }
        }
        
        // Populate with new positions
        for (i, &pos) in positions.iter().enumerate() {
            let hash = self.hash_position(pos);
            let bucket = self.buckets.entry(hash).or_insert_with(|| {
                self.bucket_pool.pop().unwrap_or_else(|| Vec::with_capacity(16))
            });
            bucket.push(i as u32);
        }
    }
    
    fn query_neighbors(&self, position: Vec3, radius: f32) -> Vec<u32> {
        self.query_neighbors(position, radius)
    }
    
    fn get_cell_size(&self) -> f32 {
        self.cell_size
    }
}