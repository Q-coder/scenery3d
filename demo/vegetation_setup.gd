extends S3DVegetationManager
##
## Demo vegetation setup. Builds a placeholder cone mesh (base at y=0,
## tip at y=1) so the unit-mesh assumption of S3DVegetationManager
## (scale.y = tree_height_m) is satisfied without further offsets.
##
## Replace `tree_mesh` from the inspector once a real conifer asset is
## available.
##

const RADIAL_SEGMENTS := 8
@export_file("*.tscn", "*.glb", "*.gltf") var tree_scene_path: String = ""

func _ready() -> void:
	print("DEBUG: vegetation_setup._ready() called")
	print("DEBUG: tree_mesh = %s" % tree_mesh)
	print("DEBUG: tree_scene_path = '%s'" % tree_scene_path)
	
	if tree_mesh == null:
		print("DEBUG: tree_mesh is null, attempting to load from path")
		if not tree_scene_path.is_empty():
			print("DEBUG: tree_scene_path is not empty, calling _load_mesh_from_scene_path")
			var loaded := _load_mesh_from_scene_path(tree_scene_path)
			if loaded != null:
				print("DEBUG: successfully loaded mesh from path")
				tree_mesh = loaded
				_brighten_mesh_materials(tree_mesh)
				return
			push_warning("Vegetation: could not extract mesh from %s, using cone fallback." % tree_scene_path)
		print("DEBUG: creating cone fallback")
		tree_mesh = _make_placeholder_cone()
	else:
		print("DEBUG: tree_mesh already set, skipping load")
		_brighten_mesh_materials(tree_mesh)


func _load_mesh_from_scene_path(path: String) -> Mesh:
	# Try to load the path. Handle both absolute paths and res:// paths.
	print("DEBUG: _load_mesh_from_scene_path() called with path: %s" % path)
	
	var load_path := path
	
	# If it's an absolute path, convert it to res:// equivalent
	# by checking if we have a symlink in the project directory
	if path.begins_with("/"):
		var filename := path.get_file()
		var res_path := "res://" + filename
		print("DEBUG: absolute path detected, trying res:// equivalent: %s" % res_path)
		var test_resource = load(res_path)
		if test_resource != null:
			print("DEBUG: found symlinked resource at %s" % res_path)
			load_path = res_path
		else:
			print("DEBUG: no symlink found, will try original absolute path")
	
	print("DEBUG: calling load(%s)" % load_path)
	var resource = load(load_path)
	print("DEBUG: load() returned: %s" % resource)
	
	if resource == null:
		push_error("Vegetation: failed to load resource: %s" % path)
		return null
	
	# Handle both PackedScene and direct Mesh resources
	if resource is PackedScene:
		print("DEBUG: resource is PackedScene, instantiating...")
		var scene := resource as PackedScene
		var root := scene.instantiate()
		var found := _find_first_mesh_recursive(root)
		print("DEBUG: found mesh: %s" % found)
		root.free()
		return found
	elif resource is Mesh:
		print("DEBUG: resource is Mesh, returning directly")
		return resource as Mesh
	else:
		push_error("Vegetation: resource %s is not a scene or mesh (type: %s)" % [path, resource.get_class()])
		return null


func _find_first_mesh_recursive(node: Node) -> Mesh:
	if node is MeshInstance3D:
		var mi := node as MeshInstance3D
		if mi.mesh != null:
			return mi.mesh

	for child in node.get_children():
		var m := _find_first_mesh_recursive(child)
		if m != null:
			return m
	return null


func _brighten_mesh_materials(mesh: Mesh) -> void:
	# Keep foliage readable without washing textures out.
	# For tree cards, force two-sided alpha-scissor rendering to avoid
	# disappearing/partial leaves when viewed from different angles.
	if mesh == null:
		return
	
	for i in range(mesh.get_surface_count()):
		var mat = mesh.surface_get_material(i)
		if mat is StandardMaterial3D:
			var std_mat := (mat as StandardMaterial3D).duplicate()
			# Keep leaves visible from both sides (common for card-based foliage).
			std_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			# Ignore incoming terrain shadows for now; dense card foliage reads
			# better than the current overly dark result.
			std_mat.set_flag(BaseMaterial3D.FLAG_DONT_RECEIVE_SHADOWS, true)
			# Use cutout transparency to stabilize leaf texture rendering.
			std_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
			std_mat.alpha_scissor_threshold = 0.35
			# Small green-tinted emission for readability, not a gray wash.
			std_mat.emission_enabled = true
			std_mat.emission = Color(0.08, 0.12, 0.08, 1.0)
			std_mat.emission_energy_multiplier = 0.25
			std_mat.roughness = clamp(std_mat.roughness, 0.5, 1.0)
			std_mat.metallic = 0.0
			mesh.surface_set_material(i, std_mat)
			print("DEBUG: adjusted foliage material surface %d" % i)
		elif mat == null:
			# No material assigned, create a simple foliage-safe fallback.
			var new_mat := StandardMaterial3D.new()
			new_mat.albedo_color = Color(0.35, 0.48, 0.32, 1.0)
			new_mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			new_mat.set_flag(BaseMaterial3D.FLAG_DONT_RECEIVE_SHADOWS, true)
			new_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA_SCISSOR
			new_mat.alpha_scissor_threshold = 0.35
			new_mat.emission_enabled = true
			new_mat.emission = Color(0.08, 0.12, 0.08, 1.0)
			new_mat.emission_energy_multiplier = 0.25
			new_mat.roughness = 0.6
			mesh.surface_set_material(i, new_mat)
			print("DEBUG: created fallback foliage material for surface %d" % i)


func _make_placeholder_cone() -> ArrayMesh:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)

	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.18, 0.40, 0.18)
	mat.roughness = 0.95
	mat.metallic = 0.0
	mat.set_flag(BaseMaterial3D.FLAG_DONT_RECEIVE_SHADOWS, true)
	st.set_material(mat)

	var apex := Vector3(0.0, 1.0, 0.0)
	var ring: Array[Vector3] = []
	for i in range(RADIAL_SEGMENTS):
		var theta := TAU * float(i) / float(RADIAL_SEGMENTS)
		ring.append(Vector3(cos(theta), 0.0, sin(theta)))

	# Side faces.
	for i in range(RADIAL_SEGMENTS):
		var a: Vector3 = ring[i]
		var b: Vector3 = ring[(i + 1) % RADIAL_SEGMENTS]
		var n: Vector3 = (b - a).cross(apex - a).normalized()
		st.set_normal(n); st.add_vertex(a)
		st.set_normal(n); st.add_vertex(b)
		st.set_normal(n); st.add_vertex(apex)

	# Bottom cap (so trees don't look hollow when seen from below).
	var down := Vector3(0.0, -1.0, 0.0)
	for i in range(1, RADIAL_SEGMENTS - 1):
		st.set_normal(down); st.add_vertex(ring[0])
		st.set_normal(down); st.add_vertex(ring[i + 1])
		st.set_normal(down); st.add_vertex(ring[i])

	return st.commit()
