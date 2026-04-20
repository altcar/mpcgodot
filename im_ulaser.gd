extends Node3D # Attached to your car

var last_pos := Vector3.ZERO
var last_rot := Quaternion.IDENTITY

func _physics_process(delta: float):
	if delta <= 0: return
	
	# 1. State extraction
	var pos = global_position
	var quat = global_transform.basis.get_rotation_quaternion()
	var lin_vel = (pos - last_pos) / delta
	var q_diff = quat * last_rot.inverse()
	var ang_vel = Vector3(q_diff.x, q_diff.y, q_diff.z) * (2.0 / delta)
	
	last_pos = pos
	last_rot = quat

	# 2. Encoding (112 bytes for 14 doubles)
	var buffer = PackedByteArray()
	buffer.resize(112)
	var data_array = [Time.get_ticks_msec()/1000.0, pos.x, pos.y, pos.z, 
					  quat.w, quat.x, quat.y, quat.z, 
					  lin_vel.x, lin_vel.y, lin_vel.z, 
					  ang_vel.x, ang_vel.y, ang_vel.z]
	
	for i in range(data_array.size()):
		buffer.encode_double(i * 8, data_array[i])

	# 3. THE FINDER: Call the group instead of a specific path
	get_tree().call_group("network_transceivers", "send_data", buffer)
