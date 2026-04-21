extends CanvasLayer

var state_data: Array = []
var lidar_data: Array = []
var lane_data: Array = []
var cte: float = 0.0
var steer: float = 0.0
var steer_hist: Array = []

@onready var panel = Control.new()
@onready var lidar_view = Control.new()
@onready var lane_view = Control.new()
@onready var steer_view = Control.new()
@onready var state_label = Label.new()
@onready var cte_label = Label.new()

func _ready():
	# UI Setup
	layer = 100
	add_child(panel)
	panel.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	panel.mouse_filter = Control.MOUSE_FILTER_IGNORE
	
	# Backdrop for state labels
	var bg = ColorRect.new()
	bg.color = Color(0, 0, 0, 0.4)
	bg.size = Vector2(250, 300)
	bg.position = Vector2(10, 10)
	panel.add_child(bg)
	
	# State Labels (Top Left)
	state_label.position = Vector2(20, 20)
	state_label.add_theme_font_size_override("font_size", 13)
	state_label.add_theme_color_override("font_color", Color.AQUA)
	panel.add_child(state_label)
	
	# Header
	var header = Label.new()
	header.text = "MPC REAL-TIME METRICS"
	header.set_anchors_and_offsets_preset(Control.PRESET_CENTER_TOP)
	header.position.y = 10
	header.add_theme_font_size_override("font_size", 18)
	header.add_theme_color_override("font_color", Color.YELLOW)
	panel.add_child(header)
	
	# CTE Label (Top Center)
	cte_label.set_anchors_and_offsets_preset(Control.PRESET_CENTER_TOP)
	cte_label.position.y = 40
	cte_label.add_theme_font_size_override("font_size", 28)
	panel.add_child(cte_label)
	
	# Lidar View (Bottom Left)
	lidar_view.position = Vector2(180, 520)
	panel.add_child(lidar_view)
	lidar_view.draw.connect(_draw_lidar)
	
	# Lane View (Bottom Right)
	lane_view.position = Vector2(750, 520)
	panel.add_child(lane_view)
	lane_view.draw.connect(_draw_lane)
	
	# Steer View (Center Right)
	steer_view.position = Vector2(1100, 300)
	panel.add_child(steer_view)
	steer_view.draw.connect(_draw_steer)

func _process(_delta):
	steer_hist.append(steer)
	if steer_hist.size() > 400: steer_hist.pop_front()
	
	# Update text
	var state_text = "MUXED STATE:\n"
	var labels = ["PosX", "PosY", "PosZ", "QuatW", "QuatX", "QuatY", "QuatZ", "LinVelX", "LinVelY", "LinVelZ", "AngVelX", "AngVelY", "AngVelZ"]
	for i in range(min(state_data.size(), labels.size())):
		state_text += "%s: %.3f\n" % [labels[i], state_data[i]]
	state_label.text = state_text
	
	cte_label.text = "CTE: %.4f m" % cte
	cte_label.add_theme_color_override("font_color", Color.GREEN if abs(cte) < 0.2 else Color.ORANGE_RED)
	
	# Trigger redraws
	lidar_view.queue_redraw()
	lane_view.queue_redraw()
	steer_view.queue_redraw()

func _draw_lidar():
	var center = Vector2.ZERO
	var radius = 150.0
	lidar_view.draw_circle(center, radius, Color(0, 0, 0, 0.4))
	lidar_view.draw_arc(center, radius, 0, TAU, 64, Color(0.2, 0.2, 0.2), 1.0)
	
	if lidar_data.is_empty(): return
	
	var points = PackedVector2Array()
	points.append(center)
	for i in range(360):
		var angle = deg_to_rad(i) - PI/2 # Offset to match Godot forward?
		var dist = lidar_data[i]
		var norm_dist = clamp(dist / 50.0, 0, 1.0)
		var p = center + Vector2(cos(angle), sin(angle)) * (norm_dist * radius)
		points.append(p)
		
		# Draw points for intensity feel
		if i % 5 == 0:
			lidar_view.draw_circle(p, 1.5, Color.LIME_GREEN)
	
	# Draw perimeter
	for i in range(1, points.size() - 1):
		lidar_view.draw_line(points[i], points[i+1], Color(0, 1, 0, 0.3), 1.0)

func _draw_lane():
	var size = Vector2(400, 150)
	lane_view.draw_rect(Rect2(-size.x/2, -size.y, size.x, size.y), Color(0, 0, 0, 0.4))
	lane_view.draw_line(Vector2(-size.x/2, 0), Vector2(size.x/2, 0), Color.GRAY, 2.0)
	
	if lane_data.is_empty(): return
	
	var step = size.x / (lane_data.size() - 1)
	var prev_p = Vector2.ZERO
	for i in range(lane_data.size()):
		var val = lane_data[i]
		var px = -size.x/2 + (i * step)
		var py = -val * size.y
		var curr_p = Vector2(px, py)
		
		if i > 0:
			lane_view.draw_line(prev_p, curr_p, Color.AQUAMARINE, 2.0)
		prev_p = curr_p
	
	# Draw CTE indicator
	var cte_x = clamp(cte * 50.0, -size.x/2, size.x/2) # 1m = 50px
	lane_view.draw_line(Vector2(cte_x, 0), Vector2(cte_x, -size.y), Color.RED, 2.0)
	lane_view.draw_string(ThemeDB.fallback_font, Vector2(cte_x + 5, -size.y + 20), "Car", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color.RED)

func _draw_steer():
	var width = 120.0
	var height = 400.0
	# Background
	steer_view.draw_rect(Rect2(-width/2, -height/2, width, height), Color(0, 0, 0, 0.4))
	# Centerline
	steer_view.draw_line(Vector2(0, -height/2), Vector2(0, height/2), Color(1, 1, 1, 0.2), 1.0)
	steer_view.draw_string(ThemeDB.fallback_font, Vector2(-width/2, -height/2 - 10), "STEERING HISTORY", HORIZONTAL_ALIGNMENT_LEFT, -1, 12, Color.YELLOW)
	
	if steer_hist.is_empty(): return
	
	var max_points = 400.0
	var step = height / max_points
	var prev_p : Vector2
	var first = true
	
	# Drawing from oldest to newest (bottom to top)
	for i in range(steer_hist.size()):
		var val = steer_hist[i]
		var px = clamp(val * 50.0, -width/2, width/2)
		var py = height/2 - (i * step)
		var curr_p = Vector2(px, py)
		
		if not first:
			steer_view.draw_line(prev_p, curr_p, Color.MAGENTA, 1.5)
		else:
			first = false
		prev_p = curr_p
