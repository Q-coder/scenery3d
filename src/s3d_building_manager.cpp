#include "s3d_building_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

// Minimal JSON parsing for manifest (no external dependency).
// We only need to parse a flat structure with string/number fields.

using namespace godot;

// ── Constructor / Destructor ────────────────────────────────────────────────

S3DBuildingManager::S3DBuildingManager()
{
	wall_material.instantiate();
	wall_material->set_albedo(Color(0.92, 0.87, 0.75)); // Warm beige.
	wall_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
	wall_material->set_diffuse_mode(StandardMaterial3D::DIFFUSE_LAMBERT_WRAP);
	wall_material->set_flag(BaseMaterial3D::FLAG_DONT_RECEIVE_SHADOWS, true);

	roof_material.instantiate();
	roof_material->set_albedo(Color(0.72, 0.38, 0.28)); // Terracotta.
	roof_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
	roof_material->set_diffuse_mode(StandardMaterial3D::DIFFUSE_LAMBERT_WRAP);
	roof_material->set_flag(BaseMaterial3D::FLAG_DONT_RECEIVE_SHADOWS, true);
}

S3DBuildingManager::~S3DBuildingManager()
{
	stop_worker();
	tiles.clear();
}

// ── Worker threads ──────────────────────────────────────────────────────────

void S3DBuildingManager::start_worker()
{
	if (worker_running.load()) return;
	worker_running.store(true);
	for (int i = 0; i < NUM_WORKERS; i++) {
		worker_threads.emplace_back(&S3DBuildingManager::worker_func, this);
	}
}

void S3DBuildingManager::stop_worker()
{
	worker_running.store(false);
	work_cv.notify_all();
	for (auto &t : worker_threads) {
		if (t.joinable()) t.join();
	}
	worker_threads.clear();
}

void S3DBuildingManager::worker_func()
{
	while (worker_running.load()) {
		LoadRequest req;
		{
			std::unique_lock<std::mutex> lock(work_mutex);
			work_cv.wait(lock, [this]() {
				return !work_queue.empty() || !worker_running.load();
			});
			if (!worker_running.load()) return;
			if (work_queue.empty()) continue;
			req = std::move(work_queue.front());
			work_queue.pop_front();
		}

		// Read GLB file.
		std::ifstream file(req.path, std::ios::binary | std::ios::ate);
		LoadResult result;
		result.tile_id = req.tile_id;

		if (!file.is_open()) {
			result.success = false;
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}

		std::streamsize size = file.tellg();
		file.seekg(0, std::ios::beg);
		std::vector<uint8_t> data(size);
		file.read(reinterpret_cast<char *>(data.data()), size);
		file.close();

		result = parse_glb(req.tile_id, data);

		std::lock_guard<std::mutex> lock(results_mutex);
		results_queue.push_back(std::move(result));
	}
}

// ── GLB Parser (runs on worker thread, produces Godot-ready arrays) ─────────

// Helper: read little-endian uint32 from buffer.
static uint32_t read_u32(const uint8_t *p) {
	return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

static float read_f32(const uint8_t *p) {
	float v;
	memcpy(&v, p, 4);
	return v;
}

// Minimal JSON value parser for manifest and glTF JSON.
// We need: objects, arrays, strings, numbers, null.
struct JsonValue {
	enum Type { NONE, OBJECT, ARRAY, STRING, NUMBER, BOOL, NUL };
	Type type = NONE;
	std::string str_val;
	double num_val = 0;
	bool bool_val = false;
	std::vector<std::pair<std::string, JsonValue>> obj_fields;
	std::vector<JsonValue> arr_items;

	const JsonValue *get(const std::string &key) const {
		for (auto &[k, v] : obj_fields) {
			if (k == key) return &v;
		}
		return nullptr;
	}

	int get_int(const std::string &key, int def = 0) const {
		auto *v = get(key);
		return (v && v->type == NUMBER) ? (int)v->num_val : def;
	}

	double get_double(const std::string &key, double def = 0) const {
		auto *v = get(key);
		return (v && v->type == NUMBER) ? v->num_val : def;
	}

	std::string get_str(const std::string &key, const std::string &def = "") const {
		auto *v = get(key);
		return (v && v->type == STRING) ? v->str_val : def;
	}
};

static void skip_ws(const char *&p, const char *end) {
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
}

static JsonValue parse_json(const char *&p, const char *end);

static std::string parse_json_string(const char *&p, const char *end) {
	if (p >= end || *p != '"') return "";
	p++; // skip opening quote
	std::string s;
	while (p < end && *p != '"') {
		if (*p == '\\' && p + 1 < end) {
			p++;
			switch (*p) {
				case '"': s += '"'; break;
				case '\\': s += '\\'; break;
				case '/': s += '/'; break;
				case 'n': s += '\n'; break;
				case 't': s += '\t'; break;
				case 'r': s += '\r'; break;
				default: s += *p; break;
			}
		} else {
			s += *p;
		}
		p++;
	}
	if (p < end) p++; // skip closing quote
	return s;
}

static JsonValue parse_json(const char *&p, const char *end) {
	skip_ws(p, end);
	if (p >= end) return {};

	JsonValue val;

	if (*p == '{') {
		val.type = JsonValue::OBJECT;
		p++;
		skip_ws(p, end);
		while (p < end && *p != '}') {
			skip_ws(p, end);
			std::string key = parse_json_string(p, end);
			skip_ws(p, end);
			if (p < end && *p == ':') p++;
			JsonValue child = parse_json(p, end);
			val.obj_fields.push_back({key, std::move(child)});
			skip_ws(p, end);
			if (p < end && *p == ',') p++;
		}
		if (p < end) p++;
	} else if (*p == '[') {
		val.type = JsonValue::ARRAY;
		p++;
		skip_ws(p, end);
		while (p < end && *p != ']') {
			val.arr_items.push_back(parse_json(p, end));
			skip_ws(p, end);
			if (p < end && *p == ',') p++;
		}
		if (p < end) p++;
	} else if (*p == '"') {
		val.type = JsonValue::STRING;
		val.str_val = parse_json_string(p, end);
	} else if (*p == 't' || *p == 'f') {
		val.type = JsonValue::BOOL;
		if (*p == 't') { val.bool_val = true; p += 4; }
		else { val.bool_val = false; p += 5; }
	} else if (*p == 'n') {
		val.type = JsonValue::NUL;
		p += 4;
	} else {
		// Number
		val.type = JsonValue::NUMBER;
		char *ep;
		val.num_val = strtod(p, &ep);
		p = ep;
	}
	return val;
}

S3DBuildingManager::LoadResult S3DBuildingManager::parse_glb(
	const std::string &tile_id, const std::vector<uint8_t> &data)
{
	LoadResult result;
	result.tile_id = tile_id;
	result.success = false;

	if (data.size() < 12) return result;

	// GLB header: magic(4) version(4) length(4)
	uint32_t magic = read_u32(&data[0]);
	if (magic != 0x46546C67) return result; // 'glTF'

	// JSON chunk
	if (data.size() < 20) return result;
	uint32_t json_len = read_u32(&data[12]);
	uint32_t json_type = read_u32(&data[16]);
	if (json_type != 0x4E4F534A) return result; // 'JSON'
	if (data.size() < 20 + json_len) return result;

	const char *json_start = reinterpret_cast<const char *>(&data[20]);
	const char *json_end = json_start + json_len;
	const char *jp = json_start;
	JsonValue gltf = parse_json(jp, json_end);

	// Binary chunk
	uint32_t bin_offset = 20 + json_len;
	// Align to 4 bytes
	bin_offset = (bin_offset + 3) & ~3;
	if (data.size() < bin_offset + 8) return result;
	uint32_t bin_len = read_u32(&data[bin_offset]);
	uint32_t bin_type = read_u32(&data[bin_offset + 4]);
	if (bin_type != 0x004E4942) return result; // 'BIN\0'
	const uint8_t *bin_data = &data[bin_offset + 8];

	// Parse accessors and buffer views.
	auto *accessors = gltf.get("accessors");
	auto *buffer_views = gltf.get("bufferViews");
	auto *meshes_json = gltf.get("meshes");
	auto *nodes_json = gltf.get("nodes");

	if (!accessors || !buffer_views || !meshes_json || !nodes_json) return result;

	// Helper lambda to read accessor data.
	auto read_vec3_accessor = [&](int acc_idx, PackedVector3Array &out) {
		if (acc_idx < 0 || acc_idx >= (int)accessors->arr_items.size()) return;
		auto &acc = accessors->arr_items[acc_idx];
		int bv_idx = acc.get_int("bufferView");
		int count = acc.get_int("count");
		if (bv_idx < 0 || bv_idx >= (int)buffer_views->arr_items.size()) return;
		auto &bv = buffer_views->arr_items[bv_idx];
		int offset = bv.get_int("byteOffset");
		out.resize(count);
		const uint8_t *p = bin_data + offset;
		for (int i = 0; i < count; i++) {
			float x = read_f32(p + i * 12);
			float y = read_f32(p + i * 12 + 4);
			float z = read_f32(p + i * 12 + 8);
			out[i] = Vector3(x, y, z);
		}
	};

	auto read_index_accessor = [&](int acc_idx, PackedInt32Array &out) {
		if (acc_idx < 0 || acc_idx >= (int)accessors->arr_items.size()) return;
		auto &acc = accessors->arr_items[acc_idx];
		int bv_idx = acc.get_int("bufferView");
		int count = acc.get_int("count");
		int comp_type = acc.get_int("componentType");
		if (bv_idx < 0 || bv_idx >= (int)buffer_views->arr_items.size()) return;
		auto &bv = buffer_views->arr_items[bv_idx];
		int offset = bv.get_int("byteOffset");
		out.resize(count);
		const uint8_t *p = bin_data + offset;
		if (comp_type == 5123) { // UNSIGNED_SHORT
			for (int i = 0; i < count; i++) {
				out[i] = p[i * 2] | (p[i * 2 + 1] << 8);
			}
		} else { // UNSIGNED_INT
			for (int i = 0; i < count; i++) {
				out[i] = read_u32(p + i * 4);
			}
		}
	};

	auto read_surface = [&](const JsonValue &prim, SurfaceData &surf) {
		auto *attrs = prim.get("attributes");
		if (attrs) {
			read_vec3_accessor(attrs->get_int("POSITION", -1), surf.vertices);
			read_vec3_accessor(attrs->get_int("NORMAL", -1), surf.normals);
		}
		read_index_accessor(prim.get_int("indices", -1), surf.indices);
	};

	// Process each node/mesh.
	for (size_t ni = 0; ni < nodes_json->arr_items.size(); ni++) {
		auto &node = nodes_json->arr_items[ni];
		std::string name = node.get_str("name");
		int mesh_idx = node.get_int("mesh", -1);
		if (mesh_idx < 0 || mesh_idx >= (int)meshes_json->arr_items.size()) continue;

		auto &mesh = meshes_json->arr_items[mesh_idx];
		auto *prims = mesh.get("primitives");
		if (!prims) continue;

		if (name == "_far_lod") {
			// Far-LOD merged mesh: 2 primitives (wall + roof).
			if (prims->arr_items.size() >= 1) {
				read_surface(prims->arr_items[0], result.far_wall);
			}
			if (prims->arr_items.size() >= 2) {
				read_surface(prims->arr_items[1], result.far_roof);
			}
			continue;
		}

		// Individual building: 4 primitives
		// [0]=wall_detail, [1]=roof_detail, [2]=wall_box, [3]=roof_box
		LoadResult::BuildingData bld;
		bld.uuid = String(name.c_str());

		if (prims->arr_items.size() >= 1)
			read_surface(prims->arr_items[0], bld.lod0_wall);
		if (prims->arr_items.size() >= 2)
			read_surface(prims->arr_items[1], bld.lod0_roof);
		if (prims->arr_items.size() >= 3)
			read_surface(prims->arr_items[2], bld.lod1_wall);
		if (prims->arr_items.size() >= 4)
			read_surface(prims->arr_items[3], bld.lod1_roof);

		result.buildings.push_back(std::move(bld));
	}

	result.success = true;
	return result;
}

// ── Manifest loading ────────────────────────────────────────────────────────

void S3DBuildingManager::load_manifest()
{
	// Ensure we only attempt this once even if the manifest is missing
	// or malformed, to avoid per-frame warning spam.
	manifest_loaded = true;

	if (buildings_path.is_empty()) return;

	String manifest_file = buildings_path + "/manifest.json";
	std::string path = std::string(manifest_file.utf8().get_data());

	std::ifstream file(path);
	if (!file.is_open()) {
		UtilityFunctions::push_warning("S3DBuildingManager: manifest.json not found at " + manifest_file);
		return;
	}

	std::string content((std::istreambuf_iterator<char>(file)),
	                     std::istreambuf_iterator<char>());
	file.close();

	const char *p = content.c_str();
	const char *end = p + content.size();
	JsonValue root = parse_json(p, end);

	auto *tiles_obj = root.get("tiles");
	if (!tiles_obj || tiles_obj->type != JsonValue::OBJECT) {
		UtilityFunctions::push_warning("S3DBuildingManager: invalid manifest.json");
		return;
	}

	manifest_origin_e = root.get_double("origin_e", 2600000.0);
	manifest_origin_n = root.get_double("origin_n", 1200000.0);

	manifest.clear();
	for (auto &[key, val] : tiles_obj->obj_fields) {
		ManifestEntry entry;
		entry.tile_id = key;
		entry.file = val.get_str("file");
		entry.center_e = val.get_double("center_e");
		entry.center_n = val.get_double("center_n");
		entry.building_count = val.get_int("building_count");
		manifest[key] = entry;
	}

	manifest_loaded = true;
	UtilityFunctions::print("S3DBuildingManager: loaded manifest with ",
	                        (int64_t)manifest.size(), " building tiles");
}

// ── Create mesh from parsed data ────────────────────────────────────────────

static Ref<ArrayMesh> make_mesh_from_arrays(
	const PackedVector3Array &verts,
	const PackedVector3Array &norms,
	const PackedInt32Array &indices,
	const Ref<StandardMaterial3D> &mat)
{
	if (verts.size() == 0 || indices.size() == 0) return Ref<ArrayMesh>();

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_NORMAL] = norms;
	arrays[Mesh::ARRAY_INDEX] = indices;

	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (mat.is_valid()) {
		mesh->surface_set_material(0, mat);
	}
	return mesh;
}

// Add a surface (wall or roof) to an existing ArrayMesh.
static void add_surface_to_mesh(
	Ref<ArrayMesh> &mesh,
	const S3DBuildingManager::SurfaceData &surf,
	const Ref<StandardMaterial3D> &mat)
{
	if (surf.vertices.size() == 0 || surf.indices.size() == 0) return;

	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = surf.vertices;
	arrays[Mesh::ARRAY_NORMAL] = surf.normals;
	arrays[Mesh::ARRAY_INDEX] = surf.indices;

	int idx = mesh->get_surface_count();
	mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	if (mat.is_valid()) {
		mesh->surface_set_material(idx, mat);
	}
}

// Create a mesh with wall + roof surfaces.
static Ref<ArrayMesh> make_wall_roof_mesh(
	const S3DBuildingManager::SurfaceData &wall,
	const S3DBuildingManager::SurfaceData &roof,
	const Ref<StandardMaterial3D> &wall_mat,
	const Ref<StandardMaterial3D> &roof_mat)
{
	bool has_wall = wall.vertices.size() > 0 && wall.indices.size() > 0;
	bool has_roof = roof.vertices.size() > 0 && roof.indices.size() > 0;
	if (!has_wall && !has_roof) return Ref<ArrayMesh>();

	Ref<ArrayMesh> mesh;
	mesh.instantiate();
	if (has_wall) add_surface_to_mesh(mesh, wall, wall_mat);
	if (has_roof) add_surface_to_mesh(mesh, roof, roof_mat);
	return mesh;
}

// ── Main update logic ───────────────────────────────────────────────────────

void S3DBuildingManager::_process(double delta)
{
	Viewport *vp = get_viewport();
	if (!vp) return;
	Camera3D *camera = vp->get_camera_3d();
	if (!camera) return;

	if (!manifest_loaded) {
		load_manifest();
		if (!manifest_loaded) return;
	}

	if (!worker_running.load()) {
		start_worker();
	}

	update_buildings(camera->get_global_position());
}

void S3DBuildingManager::update_buildings(Vector3 camera_pos)
{
	// Camera world position to LV95.
	double cam_e = origin_east - camera_pos.x;
	double cam_n = camera_pos.z + origin_north;
	last_cam_e = cam_e;
	last_cam_n = cam_n;

	// Determine which tiles are in range.
	std::unordered_set<std::string> required;
	std::vector<LoadRequest> new_requests;

	for (auto &[tid, entry] : manifest) {
		double de = entry.center_e - cam_e;
		double dn = entry.center_n - cam_n;
		double dist = std::sqrt(de * de + dn * dn);

		if (dist <= (double)far_radius_m) {
			required.insert(tid);

			auto it = tiles.find(tid);
			if (it == tiles.end()) {
				// New tile — queue load.
				TileState state;
				state.loading = true;
				tiles[tid] = state;

				std::string file_path = std::string(buildings_path.utf8().get_data())
				                        + "/" + entry.file;
				LoadRequest req;
				req.tile_id = tid;
				req.path = file_path;
				req.distance = (int)dist;
				new_requests.push_back(std::move(req));
			} else if (it->second.loaded) {
				// Update LOD based on distance.
				// LOD 0 = individual detail meshes (within detail_radius_m).
				// LOD 1+ = far merged mesh.
				int desired_lod;
				if (dist <= (double)detail_radius_m) {
					desired_lod = 0; // Individual buildings.
				} else {
					desired_lod = 1; // Far merged mesh.
				}

				TileState &state = it->second;

				// Need individual meshes but tile was loaded far-only:
				// keep far-LOD visible while re-loading with detail.
				if (desired_lod == 0 && !state.has_detail) {
					if (state.root_node) { state.root_node->queue_free(); state.root_node = nullptr; }
					// Keep far_lod_node visible as placeholder during reload.
					state.buildings.clear();
					state.loaded = false;
					state.loading = true;
					state.lod = -1;

					std::string file_path = std::string(buildings_path.utf8().get_data())
						+ "/" + entry.file;
					LoadRequest req;
					req.tile_id = tid;
					req.path = file_path;
					req.distance = (int)dist;
					new_requests.push_back(std::move(req));
					continue;
				}

				if (desired_lod != state.lod) {
					// Transitioning away from detail: free individual meshes
					// to reclaim RIDs.
					if (desired_lod >= 1 && state.has_detail) {
						if (state.root_node) {
							state.root_node->queue_free();
							state.root_node = nullptr;
						}
						state.buildings.clear();
						state.has_detail = false;
					}

					// Show/hide far LOD.
					if (state.far_lod_node) {
						state.far_lod_node->set_visible(desired_lod >= 1);
					}
					// Show/hide detail.
					if (state.root_node) {
						state.root_node->set_visible(desired_lod == 0);
					}
					state.lod = desired_lod;
				}
			}
		}
	}

	// Unload out-of-range tiles.
	double unload_dist = (double)(far_radius_m + unload_margin_m);
	std::vector<std::string> to_remove;
	for (auto &[tid, state] : tiles) {
		if (required.find(tid) == required.end()) {
			auto mit = manifest.find(tid);
			if (mit != manifest.end()) {
				double de = mit->second.center_e - cam_e;
				double dn = mit->second.center_n - cam_n;
				double dist = std::sqrt(de * de + dn * dn);
				if (dist > unload_dist) {
					to_remove.push_back(tid);
				}
			} else {
				to_remove.push_back(tid);
			}
		}
	}
	for (auto &tid : to_remove) {
		auto it = tiles.find(tid);
		if (it != tiles.end()) {
			if (it->second.root_node) it->second.root_node->queue_free();
			if (it->second.far_lod_node) it->second.far_lod_node->queue_free();
			tiles.erase(it);
		}
	}

	// Process completed loads.
	process_load_results();

	// Queue new requests (sorted by distance).
	if (!new_requests.empty()) {
		std::sort(new_requests.begin(), new_requests.end(),
			[](const LoadRequest &a, const LoadRequest &b) {
				return a.distance < b.distance;
			});
		std::lock_guard<std::mutex> lock(work_mutex);
		for (auto &req : new_requests) {
			work_queue.push_back(std::move(req));
		}
		work_cv.notify_all();
	}
}

void S3DBuildingManager::process_load_results()
{
	// Process at most a few results per frame.
	for (int budget = 0; budget < 2; budget++) {
		LoadResult result;
		{
			std::lock_guard<std::mutex> lock(results_mutex);
			if (results_queue.empty()) return;
			result = std::move(results_queue.front());
			results_queue.pop_front();
		}

		auto it = tiles.find(result.tile_id);
		if (it == tiles.end()) continue;

		TileState &state = it->second;
		state.loading = false;

		if (!result.success) {
			state.no_data = true;
			continue;
		}

		// Compute distance from camera to tile center (in LV95 metres).
		bool create_detail = false;
		auto mit = manifest.find(result.tile_id);
		if (mit != manifest.end()) {
			double de = mit->second.center_e - last_cam_e;
			double dn = mit->second.center_n - last_cam_n;
			double dist = std::sqrt(de * de + dn * dn);
			create_detail = (dist <= (double)detail_radius_m);
		}

		// Create container node for individual buildings (only if near).
		if (create_detail) {
			Node3D *root = memnew(Node3D);
			root->set_name(String("buildings_") + String(result.tile_id.c_str()));
			add_child(root);
			double dx = origin_east - manifest_origin_e;
			double dz = manifest_origin_n - origin_north;
			root->set_position(Vector3((float)dx, 0.0f, (float)dz));
			state.root_node = root;

			for (auto &bld_data : result.buildings) {
				if (bld_data.lod0_wall.vertices.size() == 0 &&
				    bld_data.lod0_roof.vertices.size() == 0) continue;

				Ref<ArrayMesh> mesh = make_wall_roof_mesh(
					bld_data.lod0_wall, bld_data.lod0_roof,
					wall_material, roof_material);

				if (mesh.is_null()) continue;

				MeshInstance3D *mi = memnew(MeshInstance3D);
				if (!bld_data.uuid.is_empty()) {
					mi->set_name(bld_data.uuid);
				}
				mi->set_mesh(mesh);
				root->add_child(mi);

				BuildingInfo info;
				info.node = mi;
				info.uuid = bld_data.uuid;

				std::string uuid_std(bld_data.uuid.utf8().get_data());
				if (hidden_buildings.find(uuid_std) != hidden_buildings.end()) {
					info.user_hidden = true;
					mi->set_visible(false);
				}

				state.buildings.push_back(std::move(info));
			}
			state.has_detail = true;
		}

		// Create far-LOD merged mesh from actual building detail geometry
		// (skip if we already have one from a previous far-only load).
		if (!state.far_lod_node) {
			SurfaceData merged_wall, merged_roof;
			int wall_offset = 0, roof_offset = 0;
			for (auto &bld_data : result.buildings) {
				auto &w = bld_data.lod0_wall;
				if (w.vertices.size() > 0 && w.indices.size() > 0) {
					int base = merged_wall.vertices.size();
					merged_wall.vertices.append_array(w.vertices);
					merged_wall.normals.append_array(w.normals);
					int idx_count = w.indices.size();
					for (int i = 0; i < idx_count; i++) {
						merged_wall.indices.push_back(w.indices[i] + base);
					}
				}
				auto &r = bld_data.lod0_roof;
				if (r.vertices.size() > 0 && r.indices.size() > 0) {
					int base = merged_roof.vertices.size();
					merged_roof.vertices.append_array(r.vertices);
					merged_roof.normals.append_array(r.normals);
					int idx_count = r.indices.size();
					for (int i = 0; i < idx_count; i++) {
						merged_roof.indices.push_back(r.indices[i] + base);
					}
				}
			}

			if (merged_wall.vertices.size() > 0 || merged_roof.vertices.size() > 0) {
				Ref<ArrayMesh> far_mesh = make_wall_roof_mesh(
					merged_wall, merged_roof,
					wall_material, roof_material);
				if (far_mesh.is_valid()) {
					MeshInstance3D *far_mi = memnew(MeshInstance3D);
					far_mi->set_name(String("_far_lod_") + String(result.tile_id.c_str()));
					far_mi->set_mesh(far_mesh);
					far_mi->set_visible(false);
					add_child(far_mi);
					double dx = origin_east - manifest_origin_e;
					double dz = manifest_origin_n - origin_north;
					far_mi->set_position(Vector3((float)dx, 0.0f, (float)dz));
					state.far_lod_node = far_mi;
				}
			}
		}

		state.loaded = true;
		state.lod = -1; // Force LOD update on next frame.
	}
}

// ── Hide/Show buildings ─────────────────────────────────────────────────────

void S3DBuildingManager::hide_building(const String &uuid)
{
	std::string key(uuid.utf8().get_data());
	hidden_buildings.insert(key);

	// Hide in any loaded tile.
	for (auto &[tid, state] : tiles) {
		for (auto &bld : state.buildings) {
			if (bld.uuid == uuid) {
				bld.user_hidden = true;
				if (bld.node) bld.node->set_visible(false);
			}
		}
	}
}

void S3DBuildingManager::show_building(const String &uuid)
{
	std::string key(uuid.utf8().get_data());
	hidden_buildings.erase(key);

	for (auto &[tid, state] : tiles) {
		for (auto &bld : state.buildings) {
			if (bld.uuid == uuid) {
				bld.user_hidden = false;
				if (bld.node && state.lod <= 1) {
					bld.node->set_visible(true);
				}
			}
		}
	}
}

bool S3DBuildingManager::is_building_hidden(const String &uuid) const
{
	std::string key(uuid.utf8().get_data());
	return hidden_buildings.find(key) != hidden_buildings.end();
}

TypedArray<String> S3DBuildingManager::get_hidden_buildings() const
{
	TypedArray<String> arr;
	for (auto &key : hidden_buildings) {
		arr.push_back(String(key.c_str()));
	}
	return arr;
}

// ── Bind methods ────────────────────────────────────────────────────────────

void S3DBuildingManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("hide_building", "uuid"), &S3DBuildingManager::hide_building);
	ClassDB::bind_method(D_METHOD("show_building", "uuid"), &S3DBuildingManager::show_building);
	ClassDB::bind_method(D_METHOD("is_building_hidden", "uuid"), &S3DBuildingManager::is_building_hidden);
	ClassDB::bind_method(D_METHOD("get_hidden_buildings"), &S3DBuildingManager::get_hidden_buildings);
	ClassDB::bind_method(D_METHOD("get_active_tile_count"), &S3DBuildingManager::get_active_tile_count);
	ClassDB::bind_method(D_METHOD("get_building_count"), &S3DBuildingManager::get_building_count);

	ClassDB::bind_method(D_METHOD("set_buildings_path", "path"), &S3DBuildingManager::set_buildings_path);
	ClassDB::bind_method(D_METHOD("get_buildings_path"), &S3DBuildingManager::get_buildings_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "buildings_path"), "set_buildings_path", "get_buildings_path");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DBuildingManager::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DBuildingManager::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DBuildingManager::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DBuildingManager::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_load_radius_m", "radius"), &S3DBuildingManager::set_load_radius_m);
	ClassDB::bind_method(D_METHOD("get_load_radius_m"), &S3DBuildingManager::get_load_radius_m);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius_m"), "set_load_radius_m", "get_load_radius_m");

	ClassDB::bind_method(D_METHOD("set_far_radius_m", "radius"), &S3DBuildingManager::set_far_radius_m);
	ClassDB::bind_method(D_METHOD("get_far_radius_m"), &S3DBuildingManager::get_far_radius_m);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "far_radius_m"), "set_far_radius_m", "get_far_radius_m");

	ClassDB::bind_method(D_METHOD("set_detail_radius_m", "radius"), &S3DBuildingManager::set_detail_radius_m);
	ClassDB::bind_method(D_METHOD("get_detail_radius_m"), &S3DBuildingManager::get_detail_radius_m);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "detail_radius_m"), "set_detail_radius_m", "get_detail_radius_m");
}

// ── Getters / Setters ───────────────────────────────────────────────────────

int S3DBuildingManager::get_active_tile_count() const
{
	int count = 0;
	for (auto &[tid, state] : tiles) {
		if (state.loaded) count++;
	}
	return count;
}

int S3DBuildingManager::get_building_count() const
{
	int count = 0;
	for (auto &[tid, state] : tiles) {
		count += (int)state.buildings.size();
	}
	return count;
}

void S3DBuildingManager::set_buildings_path(const String &p_path)
{
	buildings_path = p_path;
	manifest_loaded = false;
}

String S3DBuildingManager::get_buildings_path() const
{
	return buildings_path;
}

void S3DBuildingManager::set_origin_east(double p_east) { origin_east = p_east; }
double S3DBuildingManager::get_origin_east() const { return origin_east; }

void S3DBuildingManager::set_origin_north(double p_north) { origin_north = p_north; }
double S3DBuildingManager::get_origin_north() const { return origin_north; }

void S3DBuildingManager::set_load_radius_m(int p_radius) { load_radius_m = p_radius; }
int S3DBuildingManager::get_load_radius_m() const { return load_radius_m; }

void S3DBuildingManager::set_far_radius_m(int p_radius) { far_radius_m = p_radius; }
int S3DBuildingManager::get_far_radius_m() const { return far_radius_m; }

void S3DBuildingManager::set_detail_radius_m(int p_radius) { detail_radius_m = p_radius; }
int S3DBuildingManager::get_detail_radius_m() const { return detail_radius_m; }
