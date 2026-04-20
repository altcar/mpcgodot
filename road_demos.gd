extends Node3D

# We will look for the UDP node to get commands
# If not using Autoload, we find it in the tree
@onready var udp_node = get_tree().get_first_node_in_group("network_transceivers")

func _physics_process(delta: float):
	var steer = 0.0
	var target_speed = 0.0
	
	# 1. Get values from the UDP node if it exists
	if udp_node:
		steer = udp_node.steering_input
		target_speed = udp_node.speed_input
	
	# 2. Apply Steering (Rotate the Node3D)
	# We rotate the basis around the Y axis
	rotate_y(steer * delta)
	
	# 3. Apply Forward Movement
	var forward_vector = -global_transform.basis.z
	global_position += forward_vector * target_speed * delta
