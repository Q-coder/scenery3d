extends Camera3D

## Simple fly camera for terrain preview.
## WASD to move, mouse drag to look, scroll to change speed.

@export var move_speed: float = 200.0
@export var look_sensitivity: float = 0.002
@export var speed_step: float = 50.0

var _velocity := Vector3.ZERO
var _mouse_captured := false

func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	_mouse_captured = true

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and _mouse_captured:
		rotate_y(-event.relative.x * look_sensitivity)
		rotate_object_local(Vector3.RIGHT, -event.relative.y * look_sensitivity)

	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			move_speed = max(10.0, move_speed + speed_step)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			move_speed = max(10.0, move_speed - speed_step)

	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_ESCAPE:
			_mouse_captured = not _mouse_captured
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if _mouse_captured else Input.MOUSE_MODE_VISIBLE

func _process(delta: float) -> void:
	var dir := Vector3.ZERO
	if Input.is_key_pressed(KEY_W):
		dir -= transform.basis.z
	if Input.is_key_pressed(KEY_S):
		dir += transform.basis.z
	if Input.is_key_pressed(KEY_A):
		dir -= transform.basis.x
	if Input.is_key_pressed(KEY_D):
		dir += transform.basis.x
	if Input.is_key_pressed(KEY_Q):
		dir -= Vector3.UP
	if Input.is_key_pressed(KEY_E):
		dir += Vector3.UP

	if dir.length_squared() > 0.0:
		dir = dir.normalized()

	position += dir * move_speed * delta
