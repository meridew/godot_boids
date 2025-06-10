use std::sync::Arc;
use glam::*;
use rayon::prelude::*;
use rustc_hash::FxHashMap;

// SIMD-optimized Structure of Arrays for maximum cache performance
#[repr(C, align(64))] // Align to cache line
pub struct UltraBoidData {
    // Separate arrays for perfect vectorization
    pos_x: Vec<f32>,
    pos_y: Vec<f32>, 
    pos_z: Vec<f32>,
    vel_x: Vec<f32>,
    vel_y: Vec<f32>,
    vel_z: Vec<f32>,
    
    // Forces computed in parallel
    force_x: Vec<f32>,
    force_y: Vec<f32>,
    force_z: Vec<f32>,
    
    // Properties as separate arrays for SIMD
    max_speeds: Vec<f32>,
    max_forces: Vec<f32>,
    separations: Vec<f32>,
    alignments: Vec<f32>,
    cohesions: Vec<f32>,
    targetings: Vec<f32>,
    
    pub count: usize,
    capacity: usize,
}

impl UltraBoidData {
    pub fn with_capacity(capacity: usize) -> Self {
        // Pre-allocate everything to avoid runtime allocations
        let mut data = Self {
            pos_x: Vec::with_capacity(capacity),
            pos_y: Vec::with_capacity(capacity),
            pos_z: Vec::with_capacity(capacity),
            vel_x: Vec::with_capacity(capacity),
            vel_y: Vec::with_capacity(capacity),
            vel_z: Vec::with_capacity(capacity),
            force_x: Vec::with_capacity(capacity),
            force_y: Vec::with_capacity(capacity),
            force_z: Vec::with_capacity(capacity),
            max_speeds: Vec::with_capacity(capacity),
            max_forces: Vec::with_capacity(capacity),
            separations: Vec::with_capacity(capacity),
            alignments: Vec::with_capacity(capacity),
            cohesions: Vec::with_capacity(capacity),
            targetings: Vec::with_capacity(capacity),
            count: 0,
            capacity,
        };
        
        // Pre-fill with zeros to avoid reallocation during runtime
        data.pos_x.resize(capacity, 0.0);
        data.pos_y.resize(capacity, 0.0);
        data.pos_z.resize(capacity, 0.0);
        data.vel_x.resize(capacity, 0.0);
        data.vel_y.resize(capacity, 0.0);
        data.vel_z.resize(capacity, 0.0);
        data.force_x.resize(capacity, 0.0);
        data.force_y.resize(capacity, 0.0);
        data.force_z.resize(capacity, 0.0);
        data.max_speeds.resize(capacity, 4.0);
        data.max_forces.resize(capacity, 1.0);
        data.separations.resize(capacity, 1.2);
        data.alignments.resize(capacity, 1.5);
        data.cohesions.resize(capacity, 1.0);
        data.targetings.resize(capacity, 0.8);
        
        data
    }
    
    #[inline(always)]
    pub fn clear(&mut self) {
        self.count = 0;
        // Zero out force arrays only - reuse position/velocity/property data
        unsafe {
            std::ptr::write_bytes(self.force_x.as_mut_ptr(), 0, self.capacity);
            std::ptr::write_bytes(self.force_y.as_mut_ptr(), 0, self.capacity);
            std::ptr::write_bytes(self.force_z.as_mut_ptr(), 0, self.capacity);
        }
    }
    
    #[inline(always)]
    pub fn add_boid(&mut self, pos: Vec3, vel: Vec3, props: &crate::BoidProperties) {
        let idx = self.count;
        unsafe {
            // Unsafe for maximum performance - no bounds checking
            *self.pos_x.get_unchecked_mut(idx) = pos.x;
            *self.pos_y.get_unchecked_mut(idx) = pos.y;
            *self.pos_z.get_unchecked_mut(idx) = pos.z;
            *self.vel_x.get_unchecked_mut(idx) = vel.x;
            *self.vel_y.get_unchecked_mut(idx) = vel.y;
            *self.vel_z.get_unchecked_mut(idx) = vel.z;
            *self.max_speeds.get_unchecked_mut(idx) = props.max_speed;
            *self.max_forces.get_unchecked_mut(idx) = props.max_force;
            *self.separations.get_unchecked_mut(idx) = props.seperation;
            *self.alignments.get_unchecked_mut(idx) = props.alignment;
            *self.cohesions.get_unchecked_mut(idx) = props.cohesion;
            *self.targetings.get_unchecked_mut(idx) = props.targeting;
        }
        self.count += 1;
    }
    
    #[inline(always)]
    pub fn get_position(&self, idx: usize) -> Vec3 {
        unsafe {
            Vec3::new(
                *self.pos_x.get_unchecked(idx),
                *self.pos_y.get_unchecked(idx),
                *self.pos_z.get_unchecked(idx),
            )
        }
    }
    
    #[inline(always)]
    pub fn get_force(&self, idx: usize) -> Vec3 {
        unsafe {
            Vec3::new(
                *self.force_x.get_unchecked(idx),
                *self.force_y.get_unchecked(idx),
                *self.force_z.get_unchecked(idx),
            )
        }
    }
    
    #[inline(always)]
    pub fn set_force(&mut self, idx: usize, force: Vec3) {
        unsafe {
            *self.force_x.get_unchecked_mut(idx) = force.x;
            *self.force_y.get_unchecked_mut(idx) = force.y;
            *self.force_z.get_unchecked_mut(idx) = force.z;
        }
    }
}

// Ultra-fast spatial hash with minimal overhead
pub struct UltraSpatialHash {
    cell_size: f32,
    inv_cell_size: f32,
    buckets: FxHashMap<u64, Vec<u32>>, // u32 indices for better cache
    bucket_pool: Vec<Vec<u32>>, // Reuse vectors to avoid allocations
}

impl UltraSpatialHash {
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            inv_cell_size: 1.0 / cell_size,
            buckets: FxHashMap::default(),
            bucket_pool: Vec::with_capacity(1000),
        }
    }
    
    #[inline(always)]
    fn hash_position(&self, pos: Vec3) -> u64 {
        // Fast integer hashing for spatial coordinates
        let x = (pos.x * self.inv_cell_size) as i32;
        let y = (pos.y * self.inv_cell_size) as i32;
        let z = (pos.z * self.inv_cell_size) as i32;
        
        // Morton encoding for better spatial locality
        (((x as u64) & 0x1fffff) << 42) | 
        (((y as u64) & 0x1fffff) << 21) | 
        ((z as u64) & 0x1fffff)
    }
    
    pub fn rebuild(&mut self, boid_data: &UltraBoidData) {
        // Return all vectors to pool
        for (_, mut bucket) in self.buckets.drain() {
            bucket.clear();
            if bucket.capacity() > 0 {
                self.bucket_pool.push(bucket);
            }
        }
        
        // Populate spatial hash
        for i in 0..boid_data.count {
            let pos = boid_data.get_position(i);
            let hash = self.hash_position(pos);
            
            let bucket = self.buckets.entry(hash).or_insert_with(|| {
                self.bucket_pool.pop().unwrap_or_else(|| Vec::with_capacity(16))
            });
            bucket.push(i as u32);
        }
    }
    
    #[inline(always)]
    pub fn query_neighbors(&self, pos: Vec3, radius: f32) -> Vec<u32> {
        let mut neighbors = Vec::with_capacity(64);
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
    
    #[inline(always)]
    pub fn query_neighbors_2d(&self, pos: Vec3, radius: f32) -> Vec<u32> {
        let mut neighbors = Vec::with_capacity(64);
        let grid_radius = (radius * self.inv_cell_size).ceil() as i32;
        let center_x = (pos.x * self.inv_cell_size) as i32;
        let center_y = (pos.y * self.inv_cell_size) as i32;
        let center_z = (pos.z * self.inv_cell_size) as i32;
        
        // For 2D, only iterate over x and y
        for dx in -grid_radius..=grid_radius {
            for dy in -grid_radius..=grid_radius {
                let x = center_x + dx;
                let y = center_y + dy;
                let z = center_z; // Keep z the same for 2D
                
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

// SIMD-optimized force calculation
#[inline(always)]
pub fn calculate_forces_ultra_optimized(
    boid_data: &mut UltraBoidData,
    spatial_hash: &UltraSpatialHash,
    flock_props: &crate::FlockProperties,
    target_pos: Option<Vec3>,
) {
    let max_radius = f32::max(
        f32::max(flock_props.goal_seperation.sqrt(), flock_props.goal_alignment.sqrt()),
        flock_props.goal_cohesion.sqrt()
    );
    
    // Process in cache-friendly chunks using rayon
    const CHUNK_SIZE: usize = 512; // Tuned for L2 cache
    
    (0..boid_data.count).into_par_iter()
        .chunks(CHUNK_SIZE)
        .for_each(|chunk| {
            // Process each boid in this chunk
            for boid_idx in chunk {
                let pos = boid_data.get_position(boid_idx);
                let neighbors = spatial_hash.query_neighbors(pos, max_radius);
                
                let force = calculate_single_boid_force_optimized(
                    boid_idx, boid_data, &neighbors, flock_props, target_pos
                );
                
                // Store force directly
                boid_data.set_force(boid_idx, force);
            }
        });
}

pub fn calculate_forces_ultra_optimized_2d(
    boid_data: &mut UltraBoidData,
    spatial_hash: &UltraSpatialHash,
    flock_props: &crate::FlockProperties,
    target_pos: Option<Vec3>,
) {
    let max_radius = f32::max(
        f32::max(flock_props.goal_seperation.sqrt(), flock_props.goal_alignment.sqrt()),
        flock_props.goal_cohesion.sqrt()
    );
    
    const CHUNK_SIZE: usize = 512;
    
    (0..boid_data.count).into_par_iter()
        .chunks(CHUNK_SIZE)
        .for_each(|chunk| {
            for boid_idx in chunk {
                let pos = boid_data.get_position(boid_idx);
                let neighbors = spatial_hash.query_neighbors_2d(pos, max_radius);
                
                let force = calculate_single_boid_force_optimized(
                    boid_idx, boid_data, &neighbors, flock_props, target_pos
                );
                
                boid_data.set_force(boid_idx, force);
            }
        });
}

#[inline(always)]
fn calculate_single_boid_force_optimized(
    boid_idx: usize,
    boid_data: &UltraBoidData,
    neighbors: &[u32],
    flock_props: &crate::FlockProperties,
    target_pos: Option<Vec3>,
) -> Vec3 {
    let pos = boid_data.get_position(boid_idx);
    
    // Use SIMD-friendly accumulation
    let mut sep_sum = Vec3::ZERO;
    let mut align_sum = Vec3::ZERO;
    let mut cohere_sum = Vec3::ZERO;
    let mut counts = [0u32; 3];
    
    // Distance thresholds
    let sep_dist_sq = flock_props.goal_seperation;
    let align_dist_sq = flock_props.goal_alignment;
    let cohere_dist_sq = flock_props.goal_cohesion;
    
    // Vectorized neighbor processing
    for &neighbor_idx in neighbors {
        if neighbor_idx as usize == boid_idx { continue; }
        
        let other_pos = boid_data.get_position(neighbor_idx as usize);
        let diff = pos - other_pos;
        let dist_sq = diff.length_squared();
        
        if dist_sq < f32::EPSILON { continue; }
        
        // Branchless accumulation for better performance
        if dist_sq < sep_dist_sq {
            sep_sum += diff * (1.0 / (dist_sq * dist_sq.sqrt()));
            counts[0] += 1;
        }
        
        if dist_sq < align_dist_sq {
            unsafe {
                let vel = Vec3::new(
                    *boid_data.vel_x.get_unchecked(neighbor_idx as usize),
                    *boid_data.vel_y.get_unchecked(neighbor_idx as usize),
                    *boid_data.vel_z.get_unchecked(neighbor_idx as usize),
                );
                align_sum += vel;
            }
            counts[1] += 1;
        }
        
        if dist_sq < cohere_dist_sq {
            cohere_sum += other_pos;
            counts[2] += 1;
        }
    }
    
    // Get boid properties
    let max_speed = unsafe { *boid_data.max_speeds.get_unchecked(boid_idx) };
    let max_force = unsafe { *boid_data.max_forces.get_unchecked(boid_idx) };
    let sep_weight = unsafe { *boid_data.separations.get_unchecked(boid_idx) };
    let align_weight = unsafe { *boid_data.alignments.get_unchecked(boid_idx) };
    let cohere_weight = unsafe { *boid_data.cohesions.get_unchecked(boid_idx) };
    let target_weight = unsafe { *boid_data.targetings.get_unchecked(boid_idx) };
    
    let vel = Vec3::new(
        unsafe { *boid_data.vel_x.get_unchecked(boid_idx) },
        unsafe { *boid_data.vel_y.get_unchecked(boid_idx) },
        unsafe { *boid_data.vel_z.get_unchecked(boid_idx) },
    );
    
    let mut total_force = Vec3::ZERO;
    
    // Optimized force calculation with fast math
    if counts[0] > 0 {
        sep_sum *= 1.0 / counts[0] as f32;
        if sep_sum.length_squared() > 0.0 {
            let desired = sep_sum.normalize_or_zero() * max_speed;
            total_force += (desired - vel).clamp_length_max(max_force) * sep_weight;
        }
    }
    
    if counts[1] > 0 {
        align_sum *= 1.0 / counts[1] as f32;
        if align_sum.length_squared() > 0.0 {
            let desired = align_sum.normalize_or_zero() * max_speed;
            total_force += (desired - vel).clamp_length_max(max_force) * align_weight;
        }
    }
    
    if counts[2] > 0 {
        cohere_sum = cohere_sum * (1.0 / counts[2] as f32) - pos;
        if cohere_sum.length_squared() > 0.0 {
            let desired = cohere_sum.normalize_or_zero() * max_speed;
            total_force += (desired - vel).clamp_length_max(max_force) * cohere_weight;
        }
    }
    
    // Target following
    if let Some(target) = target_pos {
        let target_dir = target - pos;
        if target_dir.length_squared() > 0.0 {
            let desired = target_dir.normalize_or_zero() * max_speed;
            total_force += (desired - vel).clamp_length_max(max_force) * target_weight;
        }
    }
    
    total_force
}