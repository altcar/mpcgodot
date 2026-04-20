extends Node

var udp := PacketPeerUDP.new()
var listen_port := 4246
var destination_port := 4247
var destination_ip := "127.0.0.1"

# Variables to store the commands for the Car
var steering_input : float = 0.0
var speed_input : float = 0.0

func _ready():
	udp.bind(listen_port)
	add_to_group("network_transceivers")
func _process(_delta):
	while udp.get_available_packet_count() > 0:
		var packet = udp.get_packet()
		if packet.size() == 16:
			steering_input = packet.decode_double(0)
			speed_input = packet.decode_double(8)
			# DEBUG MESSAGE
			print("RECV FROM MATLAB -> Steer: %.2f, Speed: %.2f" % [steering_input, speed_input])

func send_data(buffer: PackedByteArray):
	udp.set_dest_address(destination_ip, destination_port)
	udp.put_packet(buffer)
