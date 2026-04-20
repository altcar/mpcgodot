extends Node

var udp := PacketPeerUDP.new()
var listen_port := 4246
var destination_port := 4247
var destination_ip := "127.0.0.1"

func _ready():
	# Bind to the port to listen for MATLAB
	if udp.bind(listen_port) == OK:
		print("Listening on port ", listen_port)
	else:
		print("Failed to bind port!")

func _process(_delta):
	# 1. Check for incoming packets
	if udp.get_available_packet_count() > 0:
		var packet = udp.get_packet()
		# MATLAB sent a 'double' (8 bytes), so we decode it
		var data = packet.decode_double(0) 
		print("Received from MATLAB: ", data)
		
		# 2. Transmit back to MATLAB (The Loopback)
		send_to_matlab("2")

func send_to_matlab(message: String):
	udp.set_dest_address(destination_ip, destination_port)
	var packet = message.to_utf8_buffer()
	udp.put_packet(packet)
