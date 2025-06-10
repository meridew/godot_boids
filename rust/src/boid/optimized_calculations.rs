use glam::*;
use rayon::prelude::*;
use super::{BoidData, SpatialGrid, FlockProperties};

// Optimized processing function using spatial partitioning and SoA
pub fn process_boids_optimized(
    boid_data: &mut BoidData, 
    spatial_grid: &mut SpatialGrid,
    flock_props: &FlockProperties,
    target_position: Option<Vec3>
) {
    if boid_data.count == 0 { return; }
    
    // Clear and populate spatial grid
    spatial_grid.clear();
    for i in 0..boid_data.count {
        let pos = boid_data.get_position(i);
        spatial_grid.insert(i, pos);
    }
    
    // Process boids in parallel chunks
    const CHUNK_SIZE: usize = 64; // Tune based on your CPU
    let chunks: Vec<_> = (0..boid_data.count).collect::<Vec<_>>()
        .chunks(CHUNK_SIZE)
        .map(|chunk| (chunk[0], chunk[chunk.len() - 1] + 1))
        .collect();
    
    // Calculate forces using rayon
    let forces: Vec<Vec3> = chunks.par_iter()
        .flat_map(|(start, end)| {
            calculate_chunk_forces(boid_data, spatial_grid, flock_props, target_position, *start, *end)
        })
        .collect();
    
    // Apply forces back to boid data
    for (i, force) in forces.into_iter().enumerate() {
        boid_data.set_force(i, force);
    }
}

fn calculate_chunk_forces(
    boid_data: &BoidData, 
    spatial_grid: &SpatialGrid,
    flock_props: &FlockProperties,
    target_position: Option<Vec3>,
    start: usize, 
    end: usize
) -> Vec<Vec3> {
    let mut chunk_forces = Vec::with_capacity(end - start);
    
    for i in start..end {
        let pos = boid_data.get_position(i);
        
        // Get maximum influence radius for spatial queries
        let max_radius = f32::max(
            f32::max(flock_props.goal_seperation.sqrt(), flock_props.goal_alignment.sqrt()),
            flock_props.goal_cohesion.sqrt()
        );
        
        let neighbors = spatial_grid.get_neighbors(pos, max_radius);
        let force = calculate_boid_force_soa(boid_data, flock_props, target_position, i, &neighbors);
        chunk_forces.push(force);
    }
    
    chunk_forces
}

fn calculate_boid_force_soa(
    boid_data: &BoidData, 
    flock_props: &FlockProperties,
    target_position: Option<Vec3>,
    boid_idx: usize, 
    neighbors: &[usize]
) -> Vec3 {
    let pos = boid_data.get_position(boid_idx);
    let vel = boid_data.get_velocity(boid_idx);
    let max_speed = boid_data.max_speeds[boid_idx];
    let max_force = boid_data.max_forces[boid_idx];
    let sep_weight = boid_data.separations[boid_idx];
    let align_weight = boid_data.alignments[boid_idx];
    let cohere_weight = boid_data.cohesions[boid_idx];
    let target_weight = boid_data.targetings[boid_idx];
    
    let mut steer = Vec3::ZERO;
    let mut align = Vec3::ZERO;
    let mut cohere = Vec3::ZERO;
    let mut counts = [0u32; 3]; // [steer, align, cohere]
    
    // Use flock properties for distance thresholds
    let sep_dist_sq = flock_props.goal_seperation;
    let align_dist_sq = flock_props.goal_alignment;
    let cohere_dist_sq = flock_props.goal_cohesion;
    
    for &neighbor_idx in neighbors {
        if neighbor_idx == boid_idx { continue; }
        
        let other_pos = boid_data.get_position(neighbor_idx);
        let diff = pos - other_pos;
        let dist_sq = diff.length_squared();
        
        if dist_sq < f32::EPSILON || dist_sq > cohere_dist_sq { continue; }
        
        if dist_sq < sep_dist_sq {
            let inv_sqrt_dist = 1.0 / dist_sq.sqrt();
            steer += diff * inv_sqrt_dist / dist_sq.sqrt(); // Weight by inverse distance squared
            counts[0] += 1;
        }
        
        if dist_sq < align_dist_sq {
            align += boid_data.get_velocity(neighbor_idx);
            counts[1] += 1;
        }
        
        if dist_sq < cohere_dist_sq {
            cohere += other_pos;
            counts[2] += 1;
        }
    }
    
    let mut total_force = Vec3::ZERO;
    
    // Apply separation
    if counts[0] > 0 {
        steer /= counts[0] as f32;
        if steer.length_squared() > 0.0 {
            let desired = steer.normalize() * max_speed;
            let force = (desired - vel).clamp_length_max(max_force);
            total_force += force * sep_weight;
        }
    }
    
    // Apply alignment  
    if counts[1] > 0 {
        align /= counts[1] as f32;
        if align.length_squared() > 0.0 {
            let desired = align.normalize() * max_speed;
            let force = (desired - vel).clamp_length_max(max_force);
            total_force += force * align_weight;
        }
    }
    
    // Apply cohesion
    if counts[2] > 0 {
        cohere = cohere / counts[2] as f32 - pos;
        if cohere.length_squared() > 0.0 {
            let desired = cohere.normalize() * max_speed;
            let force = (desired - vel).clamp_length_max(max_force);
            total_force += force * cohere_weight;
        }
    }
    
    // Apply target following
    if let Some(target_pos) = target_position {
        let target_dir = target_pos - pos;
        if target_dir.length_squared() > 0.0 {
            let desired = target_dir.normalize() * max_speed;
            let force = (desired - vel).clamp_length_max(max_force);
            total_force += force * target_weight;
        }
    }
    
    total_force
}

// Helper function for 2D processing (ignores Z component for spatial queries)
pub fn process_boids_2d_optimized(
    boid_data: &mut BoidData, 
    spatial_grid: &mut SpatialGrid,
    flock_props: &FlockProperties,
    target_position: Option<Vec3>
) {
    if boid_data.count == 0 { return; }
    
    // Clear and populate spatial grid
    spatial_grid.clear();
    for i in 0..boid_data.count {
        let pos = boid_data.get_position(i);
        spatial_grid.insert(i, pos);
    }
    
    // Process boids in parallel chunks
    const CHUNK_SIZE: usize = 64;
    let chunks: Vec<_> = (0..boid_data.count).collect::<Vec<_>>()
        .chunks(CHUNK_SIZE)
        .map(|chunk| (chunk[0], chunk[chunk.len() - 1] + 1))
        .collect();
    
    // Calculate forces using rayon
    let forces: Vec<Vec3> = chunks.par_iter()
        .flat_map(|(start, end)| {
            calculate_chunk_forces_2d(boid_data, spatial_grid, flock_props, target_position, *start, *end)
        })
        .collect();
    
    // Apply forces back to boid data
    for (i, force) in forces.into_iter().enumerate() {
        boid_data.set_force(i, force);
    }
}

fn calculate_chunk_forces_2d(
    boid_data: &BoidData, 
    spatial_grid: &SpatialGrid,
    flock_props: &FlockProperties,
    target_position: Option<Vec3>,
    start: usize, 
    end: usize
) -> Vec<Vec3> {
    let mut chunk_forces = Vec::with_capacity(end - start);
    
    for i in start..end {
        let pos = boid_data.get_position(i);
        
        // Get maximum influence radius for spatial queries
        let max_radius = f32::max(
            f32::max(flock_props.goal_seperation.sqrt(), flock_props.goal_alignment.sqrt()),
            flock_props.goal_cohesion.sqrt()
        );
        
        let neighbors = spatial_grid.get_neighbors_2d(pos, max_radius);
        let force = calculate_boid_force_soa(boid_data, flock_props, target_position, i, &neighbors);
        chunk_forces.push(force);
    }
    
    chunk_forces
}