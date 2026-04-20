extends Node3D

@export var speed := 10.0

func _physics_process(delta: float):
	# Move 'this' node forward relative to its own orientation
	# In Godot, -basis.z is the "forward" direction
	var forward_vector = -global_transform.basis.z
	
	# Update position: Position = Position + (Direction * Speed * Time)
	global_position += forward_vector * speed * delta
