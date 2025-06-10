use glam::*;
use rayon::prelude::*;
use rustc_hash::FxHashMap;
use super::{BoidAlgorithm, BoidInstance};
use crate::FlockProperties;

// Inline spatial hash to avoid module dependency issues
struct InlineSpatialHash {
    cell_size: f32,
    inv_cell_size: f32,
    buckets: FxHashMap<u64, Vec<u32>>,
    bucket_pool: Vec<Vec<u32>>,
}

impl InlineSpatialHash {
    fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            inv_cell_size: 1.0 / cell_size,
            buckets: FxHashMap::default(),
            bucket_pool: Vec::with_capacity(2000),
        }
    }
    
    #[inline(always)]
    fn hash_position(&self, pos: Vec3) -> u64 {
        let x = (pos.x * self.inv_cell_size) as i32;
        let y = (pos.y * self.inv_cell_size) as i32;
        let z = (pos.z * self.inv_cell_size) as i32;
        
        (((x as u64) & 0x1fffff) << 42) | 
        (((y as u64) & 0x1fffff) << 21) | 
        ((z as u64) & 0x1fffff)
    }
    
    fn rebuild_from_positions(&mut self, positions: &[(Vec3, usize)]) {
        for (_, mut bucket) in self.buckets.drain() {
            bucket.clear();
            if bucket.capacity() >= 8 && bucket.capacity() <= 64 {
                self.bucket_pool.push(bucket);
            }
        }
        
        for &(pos, index) in positions {
            let hash = self.hash_position(pos);
            let bucket = self.buckets.entry(hash).or_insert_with(|| {
                self.bucket_pool.pop().unwrap_or_else(|| Vec::with_capacity(16))
            });
            bucket.push(index as u32);
        }
    }
    
    #[inline(always)]
    fn query_neighbors(&self, pos: Vec3, radius: f32) -> Vec<u32> {
        let mut neighbors = Vec::with_capacity(128);
        let grid_radius = (radius * self.inv_cell_size).ceil() as i32;
        
        let center_x = (pos.x * self.inv_cell_size) as i32;
        let center_y = (pos.y * self.inv_cell_size) as i32;
        let center_z = (pos.z * self.inv_cell_size) as i32;
        
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
}

// Cache-aligned Structure of Arrays for SIMD processing
#[repr(C, align(64))]
pub struct UltraBoidProcessor {
    // SoA layout for vectorization
    positions_x: Vec<f32>,
    positions_y: Vec<f32>,
    positions_z: Vec<f32>,
    velocities_x: Vec<f32>,
    velocities_y: Vec<f32>,
    velocities_z: Vec<f32>,
    forces_x: Vec<f32>,
    forces_y: Vec<f32>,
    forces_z: Vec<f32>,
    
    // Properties arrays
    max_speeds: Vec<f32>,
    max_forces: Vec<f32>,
    separations: Vec<f32>,
    alignments: Vec<f32>,
    cohesions: Vec<f32>,
    targetings: Vec<f32>,
    
    spatial_hash: InlineSpatialHash,
    capacity: usize,
    count: usize,
}

impl UltraBoidProcessor {
    pub fn new(capacity: usize, cell_size: f32) -> Self {
        let mut processor = Self {
            positions_x: Vec::with_capacity(capacity),
            positions_y: Vec::with_capacity(capacity),
            positions_z: Vec::with_capacity(capacity),
            velocities_x: Vec::with_capacity(capacity),
            velocities_y: Vec::with_capacity(capacity),
            velocities_z: Vec::with_capacity(capacity),
            forces_x: Vec::with_capacity(capacity),
            forces_y: Vec::with_capacity(capacity),
            forces_z: Vec::with_capacity(capacity),
            max_speeds: Vec::with_capacity(capacity),
            max_forces: Vec::with_capacity(capacity),
            separations: Vec::with_capacity(capacity),
            alignments: Vec::with_capacity(capacity),
            cohesions: Vec::with_capacity(capacity),
            targetings: Vec::with_capacity(capacity),
            spatial_hash: InlineSpatialHash::new(cell_size),
            capacity,
            count: 0,
        };
        
        // Pre-allocate to avoid runtime allocation
        processor.resize_to_capacity();
        processor
    }
    
    #[inline(always)]
    fn resize_to_capacity(&mut self) {
        self.positions_x.resize(self.capacity, 0.0);
        self.positions_y.resize(self.capacity, 0.0);
        self.positions_z.resize(self.capacity, 0.0);
        self.velocities_x.resize(self.capacity, 0.0);
        self.velocities_y.resize(self.capacity, 0.0);
        self.velocities_z.resize(self.capacity, 0.0);
        self.forces_x.resize(self.capacity, 0.0);
        self.forces_y.resize(self.capacity, 0.0);
        self.forces_z.resize(self.capacity, 0.0);
        self.max_speeds.resize(self.capacity, 4.0);
        self.max_forces.resize(self.capacity, 1.0);
        self.separations.resize(self.capacity, 1.2);
        self.alignments.resize(self.capacity, 1.5);
        self.cohesions.resize(self.capacity, 1.0);
        self.targetings.resize(self.capacity, 0.8);
    }
    
    #[inline(always)]
    pub fn load_boids(&mut self, boids: &[BoidInstance]) {
        self.count = boids.len().min(self.capacity);
        
        // Bulk load with unsafe for maximum performance
        unsafe {
            for (i, boid) in boids.iter().take(self.count).enumerate() {
                *self.positions_x.get_unchecked_mut(i) = boid.position.x;
                *self.positions_y.get_unchecked_mut(i) = boid.position.y;
                *self.positions_z.get_unchecked_mut(i) = boid.position.z;
                *self.velocities_x.get_unchecked_mut(i) = boid.velocity.x;
                *self.velocities_y.get_unchecked_mut(i) = boid.velocity.y;
                *self.velocities_z.get_unchecked_mut(i) = boid.velocity.z;
                *self.max_speeds.get_unchecked_mut(i) = boid.properties.max_speed;
                *self.max_forces.get_unchecked_mut(i) = boid.properties.max_force;
                *self.separations.get_unchecked_mut(i) = boid.properties.seperation;
                *self.alignments.get_unchecked_mut(i) = boid.properties.alignment;
                *self.cohesions.get_unchecked_mut(i) = boid.properties.cohesion;
                *self.targetings.get_unchecked_mut(i) = boid.properties.targeting;
            }
        }
        
        // Zero out forces
        unsafe {
            std::ptr::write_bytes(self.forces_x.as_mut_ptr(), 0, self.count);
            std::ptr::write_bytes(self.forces_y.as_mut_ptr(), 0, self.count);
            std::ptr::write_bytes(self.forces_z.as_mut_ptr(), 0, self.count);
        }
    }
    
    #[inline(always)]
    pub fn store_forces(&self, boids: &mut [BoidInstance]) {
        unsafe {
            for (i, boid) in boids.iter_mut().take(self.count).enumerate() {
                boid.force = Vec3::new(
                    *self.forces_x.get_unchecked(i),
                    *self.forces_y.get_unchecked(i),
                    *self.forces_z.get_unchecked(i),
                );
            }
        }
    }
    
    #[inline(always)]
    fn get_position(&self, idx: usize) -> Vec3 {
        unsafe {
            Vec3::new(
                *self.positions_x.get_unchecked(idx),
                *self.positions_y.get_unchecked(idx),
                *self.positions_z.get_unchecked(idx),
            )
        }
    }
    
    #[inline(always)]
    fn get_velocity(&self, idx: usize) -> Vec3 {
        unsafe {
            Vec3::new(
                *self.velocities_x.get_unchecked(idx),
                *self.velocities_y.get_unchecked(idx),
                *self.velocities_z.get_unchecked(idx),
            )
        }
    }
    
    #[inline(always)]
    fn set_force(&mut self, idx: usize, force: Vec3) {
        unsafe {
            *self.forces_x.get_unchecked_mut(idx) = force.x;
            *self.forces_y.get_unchecked_mut(idx) = force.y;
            *self.forces_z.get_unchecked_mut(idx) = force.z;
        }
    }
}

impl BoidAlgorithm for UltraBoidProcessor {
    fn process_boids(&mut self, boids_data: &mut [BoidInstance], flock_props: &FlockProperties, target_pos: Option<Vec3>) {
        if boids_data.is_empty() { return; }
        
        // Load boids into SoA layout
        self.load_boids(boids_data);
        
        // Collect positions for spatial hash rebuild (avoid borrow checker issues)
        let positions: Vec<(Vec3, usize)> = (0..self.count)
            .map(|i| (self.get_position(i), i))
            .collect();
        
        // Rebuild spatial hash
        self.spatial_hash.rebuild_from_positions(&positions);
        
        // Calculate max interaction radius for spatial queries
        let max_radius = f32::max(
            f32::max(flock_props.goal_seperation.sqrt(), flock_props.goal_alignment.sqrt()),
            flock_props.goal_cohesion.sqrt()
        );
        
        // Parallel force calculation with optimal chunk size
        const CHUNK_SIZE: usize = 256; // L2 cache optimized
        
        (0..self.count).into_par_iter()
            .chunks(CHUNK_SIZE)
            .for_each(|chunk| {
                for boid_idx in chunk {
                    let force = self.calculate_boid_force(boid_idx, flock_props, target_pos, max_radius);
                    // Direct unsafe write for maximum performance
                    unsafe {
                        let processor_ptr = self as *const UltraBoidProcessor as *mut UltraBoidProcessor;
                        (*processor_ptr).set_force(boid_idx, force);
                    }
                }
            });
        
        // Store forces back to boids
        self.store_forces(boids_data);
    }
}

impl UltraBoidProcessor {
    #[inline(always)]
    fn calculate_boid_force(&self, boid_idx: usize, flock_props: &FlockProperties, target_pos: Option<Vec3>, max_radius: f32) -> Vec3 {
        let pos = self.get_position(boid_idx);
        let vel = self.get_velocity(boid_idx);
        
        // Get nearby boids from spatial hash
        let neighbors = self.spatial_hash.query_neighbors(pos, max_radius);
        
        // SIMD-friendly accumulation
        let mut sep_sum = Vec3::ZERO;
        let mut align_sum = Vec3::ZERO;
        let mut cohere_sum = Vec3::ZERO;
        let mut counts = [0u32; 3];
        
        // Distance thresholds (pre-computed)
        let sep_dist_sq = flock_props.goal_seperation;
        let align_dist_sq = flock_props.goal_alignment;
        let cohere_dist_sq = flock_props.goal_cohesion;
        
        // Vectorized neighbor processing
        for &neighbor_idx in &neighbors {
            if neighbor_idx as usize == boid_idx { continue; }
            
            let other_pos = self.get_position(neighbor_idx as usize);
            let diff = pos - other_pos;
            let dist_sq = diff.length_squared();
            
            if dist_sq < f32::EPSILON { continue; }
            
            // Separation
            if dist_sq < sep_dist_sq {
                let inv_dist_cubed = 1.0 / (dist_sq * dist_sq.sqrt());
                sep_sum += diff * inv_dist_cubed;
                counts[0] += 1;
            }
            
            // Alignment  
            if dist_sq < align_dist_sq {
                align_sum += self.get_velocity(neighbor_idx as usize);
                counts[1] += 1;
            }
            
            // Cohesion
            if dist_sq < cohere_dist_sq {
                cohere_sum += other_pos;
                counts[2] += 1;
            }
        }
        
        // Get boid properties (unsafe for speed)
        let max_speed = unsafe { *self.max_speeds.get_unchecked(boid_idx) };
        let max_force = unsafe { *self.max_forces.get_unchecked(boid_idx) };
        let sep_weight = unsafe { *self.separations.get_unchecked(boid_idx) };
        let align_weight = unsafe { *self.alignments.get_unchecked(boid_idx) };
        let cohere_weight = unsafe { *self.cohesions.get_unchecked(boid_idx) };
        let target_weight = unsafe { *self.targetings.get_unchecked(boid_idx) };
        
        let mut total_force = Vec3::ZERO;
        
        // Fast force calculation with minimal branching
        if counts[0] > 0 {
            sep_sum *= 1.0 / counts[0] as f32;
            let sep_len_sq = sep_sum.length_squared();
            if sep_len_sq > 0.0 {
                let desired = sep_sum * (max_speed / sep_len_sq.sqrt());
                total_force += (desired - vel).clamp_length_max(max_force) * sep_weight;
            }
        }
        
        if counts[1] > 0 {
            align_sum *= 1.0 / counts[1] as f32;
            let align_len_sq = align_sum.length_squared();
            if align_len_sq > 0.0 {
                let desired = align_sum * (max_speed / align_len_sq.sqrt());
                total_force += (desired - vel).clamp_length_max(max_force) * align_weight;
            }
        }
        
        if counts[2] > 0 {
            cohere_sum = cohere_sum * (1.0 / counts[2] as f32) - pos;
            let cohere_len_sq = cohere_sum.length_squared();
            if cohere_len_sq > 0.0 {
                let desired = cohere_sum * (max_speed / cohere_len_sq.sqrt());
                total_force += (desired - vel).clamp_length_max(max_force) * cohere_weight;
            }
        }
        
        // Target following
        if let Some(target) = target_pos {
            let target_dir = target - pos;
            let target_len_sq = target_dir.length_squared();
            if target_len_sq > 0.0 {
                let desired = target_dir * (max_speed / target_len_sq.sqrt());
                total_force += (desired - vel).clamp_length_max(max_force) * target_weight;
            }
        }
        
        total_force
    }
}