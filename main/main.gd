extends Node3D

@onready var boid_scene: PackedScene = preload("res://main/boid.tscn")
@onready var flock: Flock3D = $Flock
@onready var flock2: Flock3D = $Flock2
@onready var target: Marker3D = $Marker3D

func _ready():
	for i in 1000:
		var boid = boid_scene.instantiate()
		flock.add_child(boid)
		var boid2 = boid_scene.instantiate()
		flock2.add_child(boid2)
