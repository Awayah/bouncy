extends Node

var size = Vector2(1280,720);

onready var camera = preload("res://camera.gdns").new()

var active = false
var player
var camera_size
var texture_size
var image = Image.new();

signal camera_ready
signal camera_frame

signal game_start
signal force

func _ready():
	camera.set_default(0)
	camera.open();
	camera.flip(true, false);
	camera_size = Vector2(camera.get_width(), camera.get_height());
	texture_size = max(camera_size.x, camera_size.y);

	#add a function for value_changed event for HSlider
	$HSlider.connect("value_changed", self, "_onHSliderValueChanged")
	
	emit_signal("camera_ready", camera_size);

	player = get_node("/root/root/game/player");

	$animation.play("greeting_blink");

func _process(delta):
	var buffer = camera.get_image();
	if not buffer:
		return;
	image.create_from_data(texture_size, texture_size, false, Image.FORMAT_RGB8, buffer);
	emit_signal("camera_frame", image)

func _onHSliderValueChanged(value):
	# Handle changes in HSlider value
	var slider_value = $HSlider.value
	print ("slider value:", slider_value)
	camera.threshold(slider_value)
	# Use the slider value in your application logic

