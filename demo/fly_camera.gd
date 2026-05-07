extends Camera3D

## Simple fly camera for terrain preview.
## WASD to move, mouse drag to look, scroll to change speed.

@export var move_speed: float = 500.0
@export var look_sensitivity: float = 0.002
@export var speed_step: float = 50.0

var _velocity := Vector3.ZERO
var _mouse_captured := false
var _hud_label: Label
var _coords: Object
var _hud_accum := 0.0
var _ortho_visible := true
var _flat_material: StandardMaterial3D

func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	_mouse_captured = true
	_hud_label = get_node_or_null("/root/Main/HUD/Label") as Label
	# S3DCoords is registered by the scenery3d GDExtension. Origin defaults
	# to (2600000, 1200000) which matches the Scenery3D node's default world
	# origin in LV95 — the demo doesn't override it.
	if ClassDB.class_exists("S3DCoords"):
		_coords = ClassDB.instantiate("S3DCoords")
	_flat_material = StandardMaterial3D.new()
	_flat_material.albedo_color = Color(0.45, 0.55, 0.35)
	_flat_material.cull_mode = BaseMaterial3D.CULL_DISABLED

func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and _mouse_captured:
		rotate_y(-event.relative.x * look_sensitivity)
		rotate_object_local(Vector3.RIGHT, -event.relative.y * look_sensitivity)

	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			move_speed = max(1000.0, move_speed + speed_step)
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			move_speed = max(1000.0, move_speed - speed_step)

	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_ESCAPE:
			_mouse_captured = not _mouse_captured
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if _mouse_captured else Input.MOUSE_MODE_VISIBLE
		elif event.keycode == KEY_X and not event.echo:
			_ortho_visible = not _ortho_visible
			_apply_ortho_visibility()

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

	_hud_accum += delta
	if _hud_accum >= 0.1:
		_hud_accum = 0.0
		_update_hud()
		# While ortho is hidden, keep applying the override to any
		# tiles that have streamed in since the last toggle.
		if not _ortho_visible:
			_apply_ortho_visibility()

func _update_hud() -> void:
	if _hud_label == null:
		return
	var p := global_position
	var lat := 0.0
	var lon := 0.0
	var east := 0.0
	var north := 0.0
	if _coords != null:
		var lv95: Vector3 = _coords.world_to_lv95(p)
		east = lv95.x
		north = lv95.y
		var wgs: Vector3 = _coords.world_to_wgs84(p)
		lat = wgs.x
		lon = wgs.y
	var text := "lat %.5f°  lon %.5f°  alt %.0f m\n" % [lat, lon, p.y]
	text += "LV95 E %.0f  N %.0f\n" % [east, north]
	text += "Godot X %.0f  Y %.0f  Z %.0f\n" % [p.x, p.y, p.z]
	# Compass heading: per S3DCoords, Godot world axes have +X = west,
	# +Z = north. Camera forward is -basis.z; bearing is measured
	# clockwise from north, so east component is -fwd.x.
	var fwd := -global_transform.basis.z
	var heading_deg := fmod(rad_to_deg(atan2(-fwd.x, fwd.z)) + 360.0, 360.0)
	var compass_dirs := ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
	var compass: String = compass_dirs[int(round(heading_deg / 45.0)) % 8]
	# Pitch: positive = looking up.
	var horiz_len := Vector2(fwd.x, fwd.z).length()
	var pitch_deg := rad_to_deg(atan2(fwd.y, horiz_len))
	text += "heading %.0f° %s  pitch %+.0f°\n" % [heading_deg, compass, pitch_deg]
	text += "speed %.0f m/s" % move_speed
	_hud_label.text = text

# Toggle visibility of orthophoto textures by setting/clearing
# `material_override` on every S3DTile (a MeshInstance3D). The tile manager
# may add new tiles during flight, so we walk the live scene tree each time
# and also reapply when new tiles appear.
func _apply_ortho_visibility() -> void:
	var scenery := get_node_or_null("/root/Main/Scenery3D") as Node
	if scenery == null:
		return
	var override_mat: StandardMaterial3D = null if _ortho_visible else _flat_material
	_walk_apply(scenery, override_mat)

func _walk_apply(node: Node, mat: StandardMaterial3D) -> void:
	if node is MeshInstance3D:
		(node as MeshInstance3D).material_override = mat
	for child in node.get_children():
		_walk_apply(child, mat)
