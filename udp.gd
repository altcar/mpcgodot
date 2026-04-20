extends Node
# Save this as UDPManager.gd and add to Autoload

var udp := PacketPeerUDP.new()
var listen_port := 4246
var destination_port := 4247
var destination_ip := "127.0.0.1"

func _ready():
	if udp.bind(listen_port) == OK:
		print("UDP: Listening on port ", listen_port)
	else:
		print("UDP: Failed to bind port!")

func _process(_delta):
	if udp.get_available_packet_count() > 0:
		var packet = udp.get_packet()
		# If MATLAB sends a double, we decode it
		if packet.size() == 8:
			var data = packet.decode_double(0)
			print("UDP Received: ", data)
		else:
			print("UDP Received String: ", packet.get_string_from_utf8())

# This function can be called from any other script
func send_data(buffer: PackedByteArray):
	print("sending data")
	udp.set_dest_address(destination_ip, destination_port)
	udp.put_packet(buffer)
