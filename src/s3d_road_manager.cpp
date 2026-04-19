#include "s3d_road_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

// ── Constructor / Destructor ────────────────────────────────────────────────

S3DRoadManager::S3DRoadManager()
{
	road_material.instantiate();
	road_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
	road_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
	road_material->set_flag(BaseMaterial3D::FLAG_DONT_RECEIVE_SHADOWS, true);
	road_material->set_diffuse_mode(StandardMaterial3D::DIFFUSE_LAMBERT_WRAP);
}

S3DRoadManager::~S3DRoadManager()
{
	stop_worker();
	tiles.clear();
}

// ── Worker threads ──────────────────────────────────────────────────────────

void S3DRoadManager::start_worker()
{
	if (worker_running.load()) return;
	worker_running.store(true);
	for (int i = 0; i < NUM_WORKERS; i++) {
		worker_threads.emplace_back(&S3DRoadManager::worker_func, this);
	}
}

void S3DRoadManager::stop_worker()
{
	worker_running.store(false);
	work_cv.notify_all();
	for (auto &t : worker_threads) {
		if (t.joinable()) t.join();
	}
	worker_threads.clear();
}

void S3DRoadManager::worker_func()
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

		LoadResult result;
		result.tile_id = req.tile_id;

		std::ifstream file(req.path, std::ios::binary | std::ios::ate);
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

		result.success = parse_glb(data, result.surface);

		std::lock_guard<std::mutex> lock(results_mutex);
		results_queue.push_back(std::move(result));
	}
}

// ── GLB Parser ──────────────────────────────────────────────────────────────
//
// Road GLBs contain a single primitive with POSITION (vec3 float), NORMAL
// (vec3 float) and COLOR_0 (vec4 unsigned byte normalised). Indices are
// UNSIGNED_INT.

static uint32_t s3d_read_u32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float s3d_read_f32(const uint8_t *p) {
	float v;
	memcpy(&v, p, 4);
	return v;
}

// Minimal JSON value tree used for parsing the glTF JSON chunk.
namespace {
struct JsonValue {
	enum Type { NONE, OBJECT, ARRAY, STRING, NUMBER, BOOL, NUL };
	Type type = NONE;
	std::string str_val;
	double num_val = 0;
	bool bool_val = false;
	std::vector<std::pair<std::string, JsonValue>> obj_fields;
	std::vector<JsonValue> arr_items;

	const JsonValue *get(const std::string &key) const {
		for (auto &kv : obj_fields) {
			if (kv.first == key) return &kv.second;
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

void skip_ws(const char *&p, const char *end) {
	while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) p++;
}

JsonValue parse_json(const char *&p, const char *end);

std::string parse_json_string(const char *&p, const char *end) {
	if (p >= end || *p != '"') return "";
	p++;
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
	if (p < end) p++;
	return s;
}

JsonValue parse_json(const char *&p, const char *end) {
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
		val.type = JsonValue::NUMBER;
		char *ep;
		val.num_val = strtod(p, &ep);
		p = ep;
	}
	return val;
}
} // namespace

bool S3DRoadManager::parse_glb(const std::vector<uint8_t> &data, SurfaceData &out)
{
	if (data.size() < 20) return false;
	if (s3d_read_u32(&data[0]) != 0x46546C67) return false; // 'glTF'

	uint32_t json_len = s3d_read_u32(&data[12]);
	if (s3d_read_u32(&data[16]) != 0x4E4F534A) return false; // 'JSON'
	if (data.size() < 20 + json_len) return false;

	const char *json_start = reinterpret_cast<const char *>(&data[20]);
	const char *json_end = json_start + json_len;
	const char *jp = json_start;
	JsonValue gltf = parse_json(jp, json_end);

	uint32_t bin_offset = (20 + json_len + 3u) & ~3u;
	if (data.size() < bin_offset + 8) return false;
	uint32_t bin_len = s3d_read_u32(&data[bin_offset]);
	if (s3d_read_u32(&data[bin_offset + 4]) != 0x004E4942) return false; // 'BIN\0'
	const uint8_t *bin_data = &data[bin_offset + 8];
	size_t bin_max = (data.size() - (bin_offset + 8));
	if (bin_len > bin_max) bin_len = (uint32_t)bin_max;

	auto *accessors = gltf.get("accessors");
	auto *buffer_views = gltf.get("bufferViews");
	auto *meshes_json = gltf.get("meshes");
	if (!accessors || !buffer_views || !meshes_json) return false;
	if (meshes_json->arr_items.empty()) return false;

	auto &mesh = meshes_json->arr_items[0];
	auto *prims = mesh.get("primitives");
	if (!prims || prims->arr_items.empty()) return false;

	auto &prim = prims->arr_items[0];
	auto *attrs = prim.get("attributes");
	if (!attrs) return false;

	auto accessor_info = [&](int acc_idx, int &bv_offset, int &count, int &comp_type) -> bool {
		if (acc_idx < 0 || acc_idx >= (int)accessors->arr_items.size()) return false;
		auto &acc = accessors->arr_items[acc_idx];
		int bv_idx = acc.get_int("bufferView", -1);
		if (bv_idx < 0 || bv_idx >= (int)buffer_views->arr_items.size()) return false;
		auto &bv = buffer_views->arr_items[bv_idx];
		bv_offset = bv.get_int("byteOffset", 0);
		count = acc.get_int("count", 0);
		comp_type = acc.get_int("componentType", 0);
		return true;
	};

	// POSITION (VEC3 float)
	{
		int off, cnt, ct;
		if (!accessor_info(attrs->get_int("POSITION", -1), off, cnt, ct)) return false;
		out.vertices.resize(cnt);
		const uint8_t *p = bin_data + off;
		for (int i = 0; i < cnt; i++) {
			out.vertices[i] = Vector3(
				s3d_read_f32(p + i * 12 + 0),
				s3d_read_f32(p + i * 12 + 4),
				s3d_read_f32(p + i * 12 + 8));
		}
	}

	// NORMAL (VEC3 float) — optional but expected
	{
		int nrm_acc = attrs->get_int("NORMAL", -1);
		int off, cnt, ct;
		if (nrm_acc >= 0 && accessor_info(nrm_acc, off, cnt, ct)) {
			out.normals.resize(cnt);
			const uint8_t *p = bin_data + off;
			for (int i = 0; i < cnt; i++) {
				out.normals[i] = Vector3(
					s3d_read_f32(p + i * 12 + 0),
					s3d_read_f32(p + i * 12 + 4),
					s3d_read_f32(p + i * 12 + 8));
			}
		}
	}

	// COLOR_0 — VEC4 UNSIGNED_BYTE normalised (or VEC4/VEC3 float)
	{
		int col_acc = attrs->get_int("COLOR_0", -1);
		if (col_acc >= 0 && col_acc < (int)accessors->arr_items.size()) {
			auto &acc = accessors->arr_items[col_acc];
			int bv_idx = acc.get_int("bufferView", -1);
			int count = acc.get_int("count", 0);
			int comp_type = acc.get_int("componentType", 0);
			std::string acc_type = acc.get_str("type");
			int comps = (acc_type == "VEC3") ? 3 : 4;
			if (bv_idx >= 0 && bv_idx < (int)buffer_views->arr_items.size()) {
				auto &bv = buffer_views->arr_items[bv_idx];
				int offset = bv.get_int("byteOffset", 0);
				const uint8_t *p = bin_data + offset;
				out.colors.resize(count);
				if (comp_type == 5121) { // UNSIGNED_BYTE normalised
					int stride = comps;
					for (int i = 0; i < count; i++) {
						float r = p[i * stride + 0] / 255.0f;
						float g = p[i * stride + 1] / 255.0f;
						float b = p[i * stride + 2] / 255.0f;
						float a = (comps == 4) ? p[i * stride + 3] / 255.0f : 1.0f;
						out.colors[i] = Color(r, g, b, a);
					}
				} else if (comp_type == 5126) { // FLOAT
					int stride = comps * 4;
					for (int i = 0; i < count; i++) {
						float r = s3d_read_f32(p + i * stride + 0);
						float g = s3d_read_f32(p + i * stride + 4);
						float b = s3d_read_f32(p + i * stride + 8);
						float a = (comps == 4) ? s3d_read_f32(p + i * stride + 12) : 1.0f;
						out.colors[i] = Color(r, g, b, a);
					}
				}
			}
		}
	}

	// Indices
	{
		int idx_acc = prim.get_int("indices", -1);
		if (idx_acc < 0 || idx_acc >= (int)accessors->arr_items.size()) return false;
		auto &acc = accessors->arr_items[idx_acc];
		int bv_idx = acc.get_int("bufferView", -1);
		int count = acc.get_int("count", 0);
		int comp_type = acc.get_int("componentType", 0);
		if (bv_idx < 0 || bv_idx >= (int)buffer_views->arr_items.size()) return false;
		auto &bv = buffer_views->arr_items[bv_idx];
		int offset = bv.get_int("byteOffset", 0);
		out.indices.resize(count);
		const uint8_t *p = bin_data + offset;
		if (comp_type == 5123) { // UNSIGNED_SHORT
			for (int i = 0; i < count; i++) {
				out.indices[i] = p[i * 2] | (p[i * 2 + 1] << 8);
			}
		} else if (comp_type == 5125) { // UNSIGNED_INT
			for (int i = 0; i < count; i++) {
				out.indices[i] = (int)s3d_read_u32(p + i * 4);
			}
		} else if (comp_type == 5121) { // UNSIGNED_BYTE
			for (int i = 0; i < count; i++) {
				out.indices[i] = p[i];
			}
		} else {
			return false;
		}
	}

	return out.vertices.size() > 0 && out.indices.size() > 0;
}

// ── Manifest loading ────────────────────────────────────────────────────────

void S3DRoadManager::load_manifests()
{
	manifest_loaded = true;
	manifests.clear();

	PackedStringArray paths = road_paths;
	if (paths.is_empty() && !road_path.is_empty()) {
		paths.push_back(road_path);
	}
	if (paths.is_empty()) return;

	int total = 0;
	for (int i = 0; i < paths.size(); i++) {
		String dir = paths[i];
		if (dir.is_empty()) continue;

		String manifest_file = dir + "/manifest.json";
		std::string mpath = std::string(manifest_file.utf8().get_data());

		std::ifstream file(mpath);
		if (!file.is_open()) {
			UtilityFunctions::push_warning("S3DRoadManager: manifest.json not found at " + manifest_file);
			continue;
		}

		std::string content((std::istreambuf_iterator<char>(file)),
		                     std::istreambuf_iterator<char>());
		file.close();

		const char *p = content.c_str();
		const char *end = p + content.size();
		JsonValue root = parse_json(p, end);

		ManifestGroup group;
		group.path = dir;
		group.conv_origin_e = root.get_double("conversion_origin_e", origin_east);
		group.conv_origin_n = root.get_double("conversion_origin_n", origin_north);

		auto *tiles_obj = root.get("tiles");
		if (!tiles_obj || tiles_obj->type != JsonValue::OBJECT) {
			UtilityFunctions::push_warning("S3DRoadManager: invalid manifest.json at " + manifest_file);
			continue;
		}

		for (auto &kv : tiles_obj->obj_fields) {
			ManifestEntry entry;
			entry.tile_id = kv.first;
			entry.file = kv.second.get_str("file");
			entry.center_e = kv.second.get_double("center_e");
			entry.center_n = kv.second.get_double("center_n");
			entry.segments = kv.second.get_int("segments");
			group.entries[kv.first] = entry;
		}

		total += (int)group.entries.size();
		manifests.push_back(std::move(group));
	}

	UtilityFunctions::print("S3DRoadManager: loaded ",
	                        (int64_t)manifests.size(), " manifest(s), ",
	                        (int64_t)total, " road tiles");
}

// ── Update ──────────────────────────────────────────────────────────────────

void S3DRoadManager::_process(double delta)
{
	Viewport *vp = get_viewport();
	if (!vp) return;
	Camera3D *camera = vp->get_camera_3d();
	if (!camera) return;

	if (!manifest_loaded) {
		load_manifests();
		if (manifests.empty()) return;
	}

	if (!worker_running.load()) {
		start_worker();
	}

	update_tiles(camera->get_global_position());
}

void S3DRoadManager::update_tiles(Vector3 camera_pos)
{
	double cam_e = origin_east - camera_pos.x;
	double cam_n = camera_pos.z + origin_north;
	last_cam_e = cam_e;
	last_cam_n = cam_n;

	std::unordered_set<std::string> required;
	std::vector<LoadRequest> new_requests;

	for (size_t gi = 0; gi < manifests.size(); gi++) {
		const ManifestGroup &group = manifests[gi];
		for (auto &kv : group.entries) {
			const ManifestEntry &entry = kv.second;
			double de = entry.center_e - cam_e;
			double dn = entry.center_n - cam_n;
			double dist = std::sqrt(de * de + dn * dn);
			if (dist > (double)load_radius_m) continue;

			std::string key = std::to_string(gi) + ":" + entry.tile_id;
			required.insert(key);

			if (tiles.find(key) == tiles.end()) {
				TileState state;
				state.loading = true;
				tiles[key] = state;

				std::string file_path = std::string(group.path.utf8().get_data())
				                        + "/" + entry.file;
				LoadRequest req;
				req.tile_id = key;
				req.path = file_path;
				req.distance = (int)dist;
				new_requests.push_back(std::move(req));
			}
		}
	}

	// Unload.
	std::vector<std::string> to_remove;
	for (auto &kv : tiles) {
		if (required.find(kv.first) != required.end()) continue;
		// Find entry to test distance.
		size_t colon = kv.first.find(':');
		if (colon == std::string::npos) { to_remove.push_back(kv.first); continue; }
		size_t gi = (size_t)std::stoul(kv.first.substr(0, colon));
		std::string tid = kv.first.substr(colon + 1);
		if (gi >= manifests.size()) { to_remove.push_back(kv.first); continue; }
		auto mit = manifests[gi].entries.find(tid);
		if (mit == manifests[gi].entries.end()) { to_remove.push_back(kv.first); continue; }
		double de = mit->second.center_e - cam_e;
		double dn = mit->second.center_n - cam_n;
		double dist = std::sqrt(de * de + dn * dn);
		if (dist > (double)(load_radius_m + unload_margin_m))
			to_remove.push_back(kv.first);
	}
	for (auto &key : to_remove) {
		auto it = tiles.find(key);
		if (it != tiles.end()) {
			if (it->second.node) it->second.node->queue_free();
			tiles.erase(it);
		}
	}

	process_load_results();

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

void S3DRoadManager::process_load_results()
{
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

		if (!result.success
		    || result.surface.vertices.size() == 0
		    || result.surface.indices.size() == 0) {
			state.no_data = true;
			state.loaded = true;
			continue;
		}

		// Resolve the manifest group for this tile to apply the
		// conversion-origin offset.
		size_t colon = result.tile_id.find(':');
		double dx = 0.0, dz = 0.0;
		if (colon != std::string::npos) {
			size_t gi = (size_t)std::stoul(result.tile_id.substr(0, colon));
			if (gi < manifests.size()) {
				dx = origin_east - manifests[gi].conv_origin_e;
				dz = manifests[gi].conv_origin_n - origin_north;
			}
		}

		// Drape the road onto the terrain. The road GLBs carry their own
		// baked Y from whatever DTM was used at generation time, which can
		// differ from the heightmap the terrain mesh is built from and
		// produces visible gaps/clipping. Re-sample elevation at each
		// vertex's world XZ so the road always sits flush on the current
		// terrain. The small vertical_offset_m keeps roads just above
		// terrain to avoid z-fighting.
		if (elevation_db.is_valid()) {
			PackedVector3Array &verts = result.surface.vertices;
			int vn = verts.size();
			for (int i = 0; i < vn; i++) {
				Vector3 v = verts[i];
				double world_x = (double)v.x + dx;
				double world_z = (double)v.z + dz;
				double h = elevation_db->get_elevation(world_x, world_z);
				if (std::isnan(h)) h = (double)v.y; // keep original if no terrain
				v.y = (float)(h + vertical_offset_m);
				verts[i] = v;
			}
		}

		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = result.surface.vertices;
		if (result.surface.normals.size() == result.surface.vertices.size()) {
			arrays[Mesh::ARRAY_NORMAL] = result.surface.normals;
		}
		if (result.surface.colors.size() == result.surface.vertices.size()) {
			arrays[Mesh::ARRAY_COLOR] = result.surface.colors;
		}
		arrays[Mesh::ARRAY_INDEX] = result.surface.indices;

		Ref<ArrayMesh> mesh;
		mesh.instantiate();
		mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		mesh->surface_set_material(0, road_material);

		MeshInstance3D *mi = memnew(MeshInstance3D);
		mi->set_name(String("road_") + String(result.tile_id.c_str()));
		mi->set_mesh(mesh);
		// When vertices are draped, they already carry absolute ASL Y, so the
		// root node sits at y=0. Without elevation_db, keep the legacy fixed
		// offset for backward compatibility.
		float root_y = elevation_db.is_valid() ? 0.0f : (float)vertical_offset_m;
		mi->set_position(Vector3((float)dx, root_y, (float)dz));
		add_child(mi);

		state.node = mi;
		state.loaded = true;
	}
}

// ── Bind methods ────────────────────────────────────────────────────────────

void S3DRoadManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_active_tile_count"), &S3DRoadManager::get_active_tile_count);

	ClassDB::bind_method(D_METHOD("set_road_path", "path"), &S3DRoadManager::set_road_path);
	ClassDB::bind_method(D_METHOD("get_road_path"), &S3DRoadManager::get_road_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "road_path"), "set_road_path", "get_road_path");

	ClassDB::bind_method(D_METHOD("set_road_paths", "paths"), &S3DRoadManager::set_road_paths);
	ClassDB::bind_method(D_METHOD("get_road_paths"), &S3DRoadManager::get_road_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "road_paths"), "set_road_paths", "get_road_paths");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DRoadManager::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DRoadManager::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DRoadManager::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DRoadManager::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_load_radius_m", "radius"), &S3DRoadManager::set_load_radius_m);
	ClassDB::bind_method(D_METHOD("get_load_radius_m"), &S3DRoadManager::get_load_radius_m);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius_m"), "set_load_radius_m", "get_load_radius_m");

	ClassDB::bind_method(D_METHOD("set_vertical_offset_m", "offset"), &S3DRoadManager::set_vertical_offset_m);
	ClassDB::bind_method(D_METHOD("get_vertical_offset_m"), &S3DRoadManager::get_vertical_offset_m);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vertical_offset_m"), "set_vertical_offset_m", "get_vertical_offset_m");
}

// ── Getters / Setters ───────────────────────────────────────────────────────

int S3DRoadManager::get_active_tile_count() const
{
	int count = 0;
	for (auto &kv : tiles) {
		if (kv.second.loaded && !kv.second.no_data) count++;
	}
	return count;
}

void S3DRoadManager::set_road_path(const String &p_path)
{
	road_path = p_path;
	manifest_loaded = false;
}
String S3DRoadManager::get_road_path() const { return road_path; }

void S3DRoadManager::set_road_paths(const PackedStringArray &p_paths)
{
	road_paths = p_paths;
	manifest_loaded = false;
}
PackedStringArray S3DRoadManager::get_road_paths() const { return road_paths; }

void S3DRoadManager::set_origin_east(double p_east) { origin_east = p_east; }
double S3DRoadManager::get_origin_east() const { return origin_east; }

void S3DRoadManager::set_origin_north(double p_north) { origin_north = p_north; }
double S3DRoadManager::get_origin_north() const { return origin_north; }

void S3DRoadManager::set_load_radius_m(int p_radius) { load_radius_m = p_radius; }
int S3DRoadManager::get_load_radius_m() const { return load_radius_m; }

void S3DRoadManager::set_vertical_offset_m(double p_offset)
{
	vertical_offset_m = p_offset;
	// Apply immediately to already-loaded tiles.
	for (auto &kv : tiles) {
		if (kv.second.node) {
			Vector3 pos = kv.second.node->get_position();
			pos.y = (float)p_offset;
			kv.second.node->set_position(pos);
		}
	}
}
double S3DRoadManager::get_vertical_offset_m() const { return vertical_offset_m; }
