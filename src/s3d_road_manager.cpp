#include "s3d_road_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

namespace s3d {

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
	bool get_bool(const std::string &key, bool def = false) const {
		auto *v = get(key);
		if (!v) return def;
		if (v->type == BOOL) return v->bool_val;
		if (v->type == NUMBER) return v->num_val != 0;
		return def;
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
		group.elevation_baked = root.get_bool("elevation_baked", false);

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
	retry_pending_drapes();

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
		bool elevation_baked = false;
		if (colon != std::string::npos) {
			size_t gi = (size_t)std::stoul(result.tile_id.substr(0, colon));
			if (gi < manifests.size()) {
				dx = origin_east - manifests[gi].conv_origin_e;
				dz = manifests[gi].conv_origin_n - origin_north;
				elevation_baked = manifests[gi].elevation_baked;
			}
		}

		// Store the pristine surface and offsets so we can re-drape later
		// if elevation_db wasn't yet populated for the relevant terrain
		// tile when this road tile finished loading.
		// When elevation is already baked into the GLB we skip all of this.
		state.dx = dx;
		state.dz = dz;
		state.baked = result.surface;

		Ref<ArrayMesh> mesh;
		mesh.instantiate();
		state.mesh = mesh;

		// Initial surface uses baked vertices as-is; apply_drape() below
		// rebuilds with terrain-conformed Y when elevation_db is set.
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
		mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		mesh->surface_set_material(0, road_material);

		MeshInstance3D *mi = memnew(MeshInstance3D);
		mi->set_name(String("road_") + String(result.tile_id.c_str()));
		mi->set_mesh(mesh);
		// Disable shadow casting: with roads draped onto the terrain the
		// cast shadow shows up as a dark stripe on the surface that does
		// not match the terrain shading. Roads still receive shadows from
		// trees etc. via FLAG_DONT_RECEIVE_SHADOWS being unset on the
		// material — actually it IS set above, so they neither cast nor
		// receive, which is what we want for thin overlay geometry.
		mi->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
		// When elevation is baked vertex Y is already ASL+offset; always
		// place the root at y=0.  For un-baked tiles keep the legacy
		// behaviour: y=vertical_offset_m when no elevation_db is set.
		float root_y = (elevation_baked || elevation_db.is_valid()) ? 0.0f : (float)vertical_offset_m;
		mi->set_position(Vector3((float)dx, root_y, (float)dz));
		add_child(mi);

		state.node = mi;
		state.loaded = true;
		// Draping is only needed when the GLB has y=0 vertices and an
		// elevation_db is available to resolve them at runtime.
		state.drape_pending = !elevation_baked && elevation_db.is_valid();

		// Hide while drape is incomplete: leaving a partially-draped tile
		// visible drops un-resolved vertices to their baked Y (0 for the BW
		// OSM GLBs) which paints a huge black triangle from terrain altitude
		// down to sea level. The tile becomes visible once all vertices land
		// on real terrain.
		if (state.drape_pending) {
			mi->set_visible(false);
		}

		// First drape attempt (only for tiles that need it).
		if (state.drape_pending) {
			bool full = false;
			if (apply_drape(state, full)) {
				mi->set_visible(true);
				if (full) {
					state.drape_pending = false;
					state.baked = SurfaceData(); // free memory
				}
			}
		}

		// Elevation already baked → no need to keep the raw surface copy.
		if (elevation_baked) {
			state.baked = SurfaceData();
		}
	}
}

bool S3DRoadManager::apply_drape(TileState &state, bool &out_fully_resolved)
{
	out_fully_resolved = false;
	if (!elevation_db.is_valid() || state.mesh.is_null()) { out_fully_resolved = true; return true; }
	PackedVector3Array verts = state.baked.vertices;
	int vn = verts.size();
	// First pass: collect successfully-sampled heights so we can fall back
	// to their mean for vertices outside the currently-loaded DTM tiles.
	// Without this fallback the unresolved vertices kept their baked Y
	// (which is 0 in BW OSM data) and produced huge black wedges from
	// terrain altitude down to sea level.
	double sum_h = 0.0;
	int n_resolved = 0;
	PackedFloat64Array sampled;
	sampled.resize(vn);
	for (int i = 0; i < vn; i++) {
		Vector3 v = verts[i];
		double world_x = (double)v.x + state.dx;
		double world_z = (double)v.z + state.dz;
		double h = elevation_db->get_elevation(world_x, world_z);
		sampled[i] = h;
		if (!std::isnan(h)) {
			sum_h += h;
			n_resolved++;
		}
	}
	if (n_resolved == 0) {
		// Nothing in the elevation DB yet for this tile.
		state.drape_failures++;
		if (!state.warned) {
			state.warned = true;
			double v0x = vn > 0 ? (double)verts[0].x + state.dx : 0;
			double v0z = vn > 0 ? (double)verts[0].z + state.dz : 0;
			int tx = (int)std::floor((elevation_db->get_origin_east() - v0x) / (double)elevation_db->get_tile_size());
			int tz = (int)std::floor((v0z + elevation_db->get_origin_north()) / (double)elevation_db->get_tile_size());
			bool has = elevation_db->has_tile(tx, tz);
			UtilityFunctions::print("S3DRoadManager: drape pending, no DTM samples; vn=",
				(int64_t)vn, " first world=(", v0x, ",", v0z, ") wants tile (", tx, ",", tz, ") has=", has);
		}
		// Fallback: after several failed retries, show the tile flat at the
		// last-known terrain Y so it's at least visible. The tile stays
		// pending so future DTM arrivals can refine it.
		if (state.drape_failures >= 4 && !std::isnan(last_known_terrain_y)) {
			for (int i = 0; i < vn; i++) {
				Vector3 v = verts[i];
				v.y = (float)(last_known_terrain_y + vertical_offset_m);
				verts[i] = v;
			}
			Array arrays;
			arrays.resize(Mesh::ARRAY_MAX);
			arrays[Mesh::ARRAY_VERTEX] = verts;
			if (state.baked.normals.size() == vn) arrays[Mesh::ARRAY_NORMAL] = state.baked.normals;
			if (state.baked.colors.size() == vn) arrays[Mesh::ARRAY_COLOR] = state.baked.colors;
			arrays[Mesh::ARRAY_INDEX] = state.baked.indices;
			state.mesh->clear_surfaces();
			state.mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
			state.mesh->surface_set_material(0, road_material);
			out_fully_resolved = false;
			return true;
		}
		return false;
	}
	double fallback_h = sum_h / (double)n_resolved;
	last_known_terrain_y = fallback_h;
	for (int i = 0; i < vn; i++) {
		Vector3 v = verts[i];
		double h = sampled[i];
		if (std::isnan(h)) h = fallback_h;
		v.y = (float)(h + vertical_offset_m);
		verts[i] = v;
	}
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	if (state.baked.normals.size() == vn) arrays[Mesh::ARRAY_NORMAL] = state.baked.normals;
	if (state.baked.colors.size() == vn) arrays[Mesh::ARRAY_COLOR] = state.baked.colors;
	arrays[Mesh::ARRAY_INDEX] = state.baked.indices;
	state.mesh->clear_surfaces();
	state.mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	state.mesh->surface_set_material(0, road_material);
	// Report whether the drape was fully resolved — partial drapes will be
	// retried when new elevation tiles arrive so the fallback mean is
	// eventually replaced by real DTM samples.
	out_fully_resolved = (n_resolved == vn);
	return true; // presentable: at least one vertex resolved
}

void S3DRoadManager::retry_pending_drapes()
{
	if (!elevation_db.is_valid()) return;
	// Skip entirely when no new terrain has been loaded since last retry —
	// re-running apply_drape (which rebuilds the GPU mesh) every frame for
	// stuck-pending tiles was the main perf regression.
	uint64_t cur = elevation_db->get_epoch();
	if (cur == elevation_epoch_seen) return;
	elevation_epoch_seen = cur;
	// Retry every pending tile in one pass. The epoch gate above ensures
	// this only runs when the DB has actually gained new tiles, so the
	// GPU re-upload spike happens at most once per ingest burst (not per
	// frame) — far more responsive than the previous 1-tile-per-frame
	// throttle, which left tiles draped at stale fallback Y for many
	// seconds whenever a partial-resolve tile sat at the iteration head.
	for (auto &kv : tiles) {
		TileState &st = kv.second;
		if (!st.drape_pending || !st.loaded || st.node == nullptr) continue;
		bool full = false;
		if (apply_drape(st, full)) {
			st.node->set_visible(true);
			if (full) {
				st.drape_pending = false;
				st.baked = SurfaceData();
			}
		}
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

} // namespace s3d
