extends Node3D

@export var max_range := 50.0 # Maximum distance of the laser

var last_pos := Vector3.ZERO
var last_rot := Quaternion.IDENTITY

func _ready():
	# Ensure variables are initialized with current global state
	last_pos = global_position
	last_rot = global_transform.basis.get_rotation_quaternion()

func _physics_process(delta: float):
	if delta <= 0: return
	
	# --- 1. State Extraction ---
	var pos = global_position
	var curr_basis = global_transform.basis
	var quat = curr_basis.get_rotation_quaternion()
	
	# Calculate velocities
	var lin_vel = (pos - last_pos) / delta
	var q_diff = quat * last_rot.inverse()
	var ang_vel = Vector3(q_diff.x, q_diff.y, q_diff.z) * (2.0 / delta)
	
	last_pos = pos
	last_rot = quat

	# --- 2. 360 Degree Laser Scan ---
	var scan_data = []
	var space_state = get_world_3d().direct_space_state
	
	# Safety check for physics state
	if not space_state:
		return

	for i in range(360):
		var angle = deg_to_rad(i)
		# Create a direction vector (pointing out horizontally)
		var local_dir = Vector3(sin(angle), 0, cos(angle))
		
		# Rotate the local direction by the car's current global orientation
		var global_dir = curr_basis * local_dir 

		var ray_end = pos + global_dir * max_range
		var query = PhysicsRayQueryParameters3D.create(pos, ray_end)
		
		# Exclude the car itself to avoid self-collision
		var parent_node = get_parent()
		if parent_node is CollisionObject3D:
			query.exclude = [parent_node.get_rid()]
		
		var result = space_state.intersect_ray(query)
		if result:
			scan_data.append(pos.distance_to(result.position))
		else:
			scan_data.append(max_range)

	# --- 3. Encoding (374 doubles = 2992 bytes) ---
	var buffer = PackedByteArray()
	buffer.resize(374 * 8) 
	
	var state_array = [
		Time.get_ticks_msec()/1000.0, pos.x, pos.y, pos.z, 
		quat.w, quat.x, quat.y, quat.z, 
		lin_vel.x, lin_vel.y, lin_vel.z, 
		ang_vel.x, ang_vel.y, ang_vel.z
	]
	
	# Pack IMU data
	for i in range(state_array.size()):
		buffer.encode_double(i * 8, state_array[i])
		
	# Pack Laser data starting at offset 112 (14 * 8 bytes)
	for i in range(scan_data.size()):
		buffer.encode_double(112 + (i * 8), scan_data[i])

	# --- 4. Send to UDP Manager ---
	get_tree().call_group("network_transceivers", "send_data", buffer)
