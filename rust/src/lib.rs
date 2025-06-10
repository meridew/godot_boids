use glam::*;
use godot::{classes::Engine, prelude::*};
use indexmap::IndexMap;
use rustc_hash::FxBuildHasher;

mod algorithms;
mod boid;
mod flock;

pub use algorithms::*;
pub use boid::*;
pub use flock::*;

type FxIndexMap<K, V> = IndexMap<K, V, FxBuildHasher>;

const SINGLETON_NAME: &str = "Boids";

fn get_singleton() -> Gd<Boids> {
    Engine::singleton()
        .get_singleton(SINGLETON_NAME)
        .unwrap()
        .cast()
}

struct BoidsExtension;

#[gdextension]
unsafe impl ExtensionLibrary for BoidsExtension {
    fn on_level_init(level: InitLevel) {
        match level {
            InitLevel::Scene => {
                let singleton = Boids::new_alloc();
                Engine::singleton().register_singleton(SINGLETON_NAME, &singleton);
            }
            _ => (),
        }
    }

    fn on_level_deinit(level: InitLevel) {
        if level == InitLevel::Scene {
            let mut engine = Engine::singleton();
            let singleton = engine
                .get_singleton(SINGLETON_NAME)
                .expect("cannot retrieve the singleton");
            engine.unregister_singleton(SINGLETON_NAME);
            singleton.free();
        }
    }
}

#[derive(GodotClass)]
#[class(init, base=Node)]
pub struct BoidsProcess {
    #[export]
    #[init(val = true)]
    process_2d: bool,
    #[export]
    #[init(val = true)]
    process_3d: bool,
    #[export]
    #[init(val = 1)]
    process_per_tick: i64,
    boids: Option<Gd<Boids>>,
    engine: Option<Gd<Engine>>,
}

#[godot_api]
impl INode for BoidsProcess {
    fn ready(&mut self) {
        self.boids = Some(get_singleton());
        self.engine = Some(Engine::singleton());
    }

    fn physics_process(&mut self, _: f64) {
        if self.engine.as_ref().unwrap().get_physics_frames() % (self.process_per_tick as u64) == 0 {
            let mut s = self.boids.as_mut().unwrap().bind_mut();
            if self.process_2d {
                s.process_boids_2d();
            }
            if self.process_3d {
                s.process_boids_3d();
            }
        }
    }
}

#[derive(GodotClass)]
#[class(init, base=Object)]
struct Boids {
    #[init(val = FxIndexMap::default())]
    flocks2d: FxIndexMap<InstanceId, Gd<Flock2D>>,
    #[init(val = FxIndexMap::default())]
    boids2d: FxIndexMap<InstanceId, Gd<Boid2D>>,
    #[init(val = FxIndexMap::default())]
    flocks3d: FxIndexMap<InstanceId, Gd<Flock3D>>,
    #[init(val = FxIndexMap::default())]
    boids3d: FxIndexMap<InstanceId, Gd<Boid3D>>,
    
    // Ultra-performance processors
    #[init(val = UltraBoidProcessor::new(15000, 75.0))]
    processor_2d: UltraBoidProcessor,
    #[init(val = UltraBoidProcessor::new(15000, 50.0))]
    processor_3d: UltraBoidProcessor,
    
    base: Base<Object>,
}

impl Boids {
    fn register_flock_2d(&mut self, flock_id: InstanceId) {
        let flock = Gd::from_instance_id(flock_id);
        self.flocks2d.insert(flock_id, flock);
    }

    fn unregister_flock_2d(&mut self, flock_id: InstanceId) {
        self.flocks2d.shift_remove(&flock_id);
    }

    fn register_boid_2d(&mut self, boid_id: InstanceId, boid: Gd<Boid2D>) {
        self.boids2d.insert(boid_id, boid);
    }

    fn unregister_boid_2d(&mut self, boid_id: InstanceId) {
        self.boids2d.shift_remove(&boid_id);
    }

    fn register_flock_3d(&mut self, flock_id: InstanceId) {
        let flock = Gd::from_instance_id(flock_id);
        self.flocks3d.insert(flock_id, flock);
    }

    fn unregister_flock_3d(&mut self, flock_id: InstanceId) {
        self.flocks3d.shift_remove(&flock_id);
    }

    fn register_boid_3d(&mut self, boid_id: InstanceId, boid: Gd<Boid3D>) {
        self.boids3d.insert(boid_id, boid);
    }

    fn unregister_boid_3d(&mut self, boid_id: InstanceId) {
        self.boids3d.shift_remove(&boid_id);
    }
}

#[godot_api]
impl Boids {
    #[func]
    fn process_boids_2d(&mut self) {
        process_boids_ultra_2d(&mut self.boids2d, &self.flocks2d, &mut self.processor_2d);
    }

    #[func]
    fn process_boids_3d(&mut self) {
        process_boids_ultra_3d(&mut self.boids3d, &self.flocks3d, &mut self.processor_3d);
    }

    #[func]
    fn get_total_boid_2d_count(&self) -> i64 {
        self.boids2d.len() as i64
    }

    #[func]
    fn get_total_flock_2d_count(&self) -> i64 {
        self.flocks2d.len() as i64
    }

    #[func]
    fn get_total_boid_3d_count(&self) -> i64 {
        self.boids3d.len() as i64
    }

    #[func]
    fn get_total_flock_3d_count(&self) -> i64 {
        self.flocks3d.len() as i64
    }
}

#[inline(always)]
const fn to_glam_vec(godot_vec: Vector3) -> Vec3 {
    vec3(godot_vec.x, godot_vec.y, godot_vec.z)
}

// Ultra-performance processing functions
fn process_boids_ultra_2d(
    boids: &mut FxIndexMap<InstanceId, Gd<Boid2D>>,
    flocks: &FxIndexMap<InstanceId, Gd<Flock2D>>,
    processor: &mut UltraBoidProcessor,
) {
    if flocks.is_empty() { return; }
    
    // Collect all boids into algorithm-friendly format
    let mut boid_instances = Vec::with_capacity(boids.len());
    let mut boid_ids = Vec::with_capacity(boids.len());
    let mut flock_props = None;
    let mut target_pos = None;
    
    for (_, flock) in flocks.iter() {
        let flock = flock.bind();
        if !flock.is_boid_processing() { continue; }
        
        // For simplicity, use first flock's properties
        // (you can extend this for multiple flocks)
        if flock_props.is_none() {
            flock_props = Some(flock.get_flock_properties().clone());
            target_pos = flock.get_target_position();
        }
        
        for (boid_id, (pos, vel, props)) in flock.get_boids() {
            boid_instances.push(BoidInstance::new(pos, vel, props));
            boid_ids.push(*boid_id);
        }
    }
    
    if boid_instances.is_empty() { return; }
    
    // Process with ultra-performance algorithm
    if let Some(props) = flock_props {
        processor.process_boids(&mut boid_instances, &props, target_pos);
    }
    
    // Apply forces back to Godot objects
    for (i, boid_id) in boid_ids.iter().enumerate() {
        if let Some(boid) = boids.get_mut(boid_id) {
            boid.bind_mut().apply_force(boid_instances[i].force);
        }
    }
}

fn process_boids_ultra_3d(
    boids: &mut FxIndexMap<InstanceId, Gd<Boid3D>>,
    flocks: &FxIndexMap<InstanceId, Gd<Flock3D>>,
    processor: &mut UltraBoidProcessor,
) {
    if flocks.is_empty() { return; }
    
    let mut boid_instances = Vec::with_capacity(boids.len());
    let mut boid_ids = Vec::with_capacity(boids.len());
    let mut flock_props = None;
    let mut target_pos = None;
    
    for (_, flock) in flocks.iter() {
        let flock = flock.bind();
        if !flock.is_boid_processing() { continue; }
        
        if flock_props.is_none() {
            flock_props = Some(flock.get_flock_properties().clone());
            target_pos = flock.get_target_position();
        }
        
        for (boid_id, (pos, vel, props)) in flock.get_boids() {
            boid_instances.push(BoidInstance::new(pos, vel, props));
            boid_ids.push(*boid_id);
        }
    }
    
    if boid_instances.is_empty() { return; }
    
    if let Some(props) = flock_props {
        processor.process_boids(&mut boid_instances, &props, target_pos);
    }
    
    for (i, boid_id) in boid_ids.iter().enumerate() {
        if let Some(boid) = boids.get_mut(boid_id) {
            boid.bind_mut().apply_force(boid_instances[i].force);
        }
    }
}