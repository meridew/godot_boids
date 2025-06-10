use super::*;
use crate::{get_singleton, BoidProperties, FlockProperties, FxIndexMap};

#[derive(GodotClass)]
#[class(init, base=Node2D)]
pub struct Boid2D {
    #[export]
    properties: Option<Gd<BoidProperties>>,
    props: BoidProperties,
    vel: Vec2,
    flock_id: Option<InstanceId>,
    base: Base<Node2D>,
}

#[godot_api]
impl Boid2D {
    #[func]
    #[inline(always)]
    fn get_velocity(&self) -> Vector2 {
        Vector2::new(self.vel.x, self.vel.y)
    }

    #[func]
    #[inline(always)]
    fn set_velocity(&mut self, new_velocity: Vector2) {
        self.vel.x = new_velocity.x;
        self.vel.y = new_velocity.y;
    }

    #[func]
    #[inline(always)]
    pub fn get_id(&self) -> InstanceId {
        self.base().instance_id()
    }

    #[func]
    #[inline(always)]
    pub fn get_flock_id(&self) -> InstanceId {
        self.flock_id.expect("no flock id set... this is a bug!")
    }
}

#[godot_api]
impl INode2D for Boid2D {
    fn enter_tree(&mut self) {
        let Some(mut flock) = self
            .to_gd()
            .get_parent()
            .and_then(|gd| gd.try_cast::<Flock2D>().ok())
        else {
            let boid_id = self.get_id();
            godot_error!("[Boid2D:{boid_id}] boids parent isn't a Flock2D, or has no parent");
            return;
        };
        let mut flock = flock.bind_mut();
        flock.register_boid(self.get_id());
        self.flock_id = Some(flock.get_id());
    }

    fn ready(&mut self) {
        if let Some(props) = self.properties.as_ref() {
            self.props = props.bind().clone();
        }
    }

    fn exit_tree(&mut self) {
        let mut flock: Gd<Flock2D> = Gd::from_instance_id(self.get_flock_id());
        flock.bind_mut().unregister_boid(self.get_id());
    }
}

impl Boid for Boid2D {
    #[inline(always)]
    fn apply_force(&mut self, force: Vec3) {
        self.vel += force.xy();
        self.vel = self.vel.clamp_length_max(self.props.max_speed);
        let force_to_apply = Vector2::new(self.vel.x, self.vel.y);
        self.base_mut().translate(force_to_apply);
    }

    #[inline(always)]
    fn get_boid_position(&self) -> Vec3 {
        let pos = self.base().get_position();
        vec3(pos.x, pos.y, 0.0)
    }

    #[inline(always)]
    fn get_boid_velocity(&self) -> Vec3 {
        vec3(self.vel.x, self.vel.y, 0.0)
    }

    #[inline(always)]
    fn get_boid_properties(&self) -> &BoidProperties {
        &self.props
    }

    #[inline(always)]
    fn get_flock_id(&self) -> InstanceId {
        self.get_flock_id()
    }
}

// Flock2D implementation
use crate::flock::Flock;

#[derive(GodotClass)]
#[class(init, base=Node2D)]
pub struct Flock2D {
    #[export]
    properties: Option<Gd<FlockProperties>>,
    props: FlockProperties,
    #[export]
    target: Option<Gd<Node2D>>,
    #[export]
    #[init(val = true)]
    boid_processing_enabled: bool,
    pub boids: FxIndexMap<InstanceId, Gd<Boid2D>>,
    base: Base<Node2D>,
}

impl Flock2D {
    pub fn register_boid(&mut self, boid_id: InstanceId) {
        let boid: Gd<Boid2D> = Gd::from_instance_id(boid_id);
        self.boids.insert(boid_id, boid.clone());
        get_singleton().bind_mut().register_boid_2d(boid_id, boid);
    }

    pub fn unregister_boid(&mut self, boid_id: InstanceId) {
        self.boids.shift_remove(&boid_id);
        get_singleton().bind_mut().unregister_boid_2d(boid_id);
    }
}

#[godot_api]
impl INode2D for Flock2D {
    fn enter_tree(&mut self) {
        get_singleton().bind_mut().register_flock_2d(self.get_id())
    }

    fn ready(&mut self) {
        if let Some(props) = self.properties.as_ref() {
            self.props = props.bind().clone();
        }
    }

    fn exit_tree(&mut self) {
        get_singleton().bind_mut().unregister_flock_2d(self.get_id())
    }
}

#[godot_api]
impl Flock2D {
    #[func]
    pub fn get_id(&self) -> InstanceId {
        self.base().instance_id()
    }
}

impl Flock for Flock2D {
    fn get_flock_properties(&self) -> &FlockProperties {
        &self.props
    }

    fn get_target_position(&self) -> Option<Vec3> {
        self.target.as_ref().map(|t| {
            let pos = t.get_position();
            vec3(pos.x, pos.y, 0.0)
        })
    }

    fn get_boids_posvel(&self) -> Vec<(Vec3, Vec3)> {
        let boid_count = self.boids.len();
        let mut result = Vec::with_capacity(boid_count);
        result.extend(self.boids.values().map(|b| {
            let b = b.bind();
            (b.get_boid_position(), b.get_boid_velocity())
        }));
        result
    }

    fn get_boids(&self) -> impl Iterator<Item = (&InstanceId, (Vec3, Vec3, BoidProperties))> {
        self.boids.iter().map(|(id, boid)| {
            let boid = boid.bind();
            (
                id,
                (
                    boid.get_boid_position(),
                    boid.get_boid_velocity(),
                    boid.get_boid_properties().clone(),
                ),
            )
        })
    }
    
    fn is_boid_processing(&self) -> bool {
        self.boid_processing_enabled
    }
}