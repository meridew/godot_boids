extends Node3D

@onready var boid_scene: PackedScene = preload("res://main/boid.tscn")
@onready var flock: Flock3D = $Flock
@onready var flock2: Flock3D = $Flock2
@onready var target: Marker3D = $Marker3D

# Define spawn area
const spawn_area := Vector3(20.0, 20.0, 20.0)

func _ready():
	# Add debug camera
	DebugCam.add_debug_cam(self)
	
	# Spawn boids with random positions
	for i in 1000:  # Reduced count for better performance
		spawn_boid(flock)
		spawn_boid(flock2)

func spawn_boid(target_flock: Flock3D) -> void:
	var boid = boid_scene.instantiate()
	# Give each boid a random position within the spawn area
	boid.global_position = Vector3(
		randf_range(-spawn_area.x, spawn_area.x),
		randf_range(-spawn_area.y, spawn_area.y), 
		randf_range(-spawn_area.z, spawn_area.z)
	)
	target_flock.add_child(boid)
