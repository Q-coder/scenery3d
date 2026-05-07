#include "s3d_vegetation_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

// File format constants (must match tools/extract_tlm_forests.py).
static constexpr uint32_t VEG_MAGIC = 0x31474556; // 'V','E','G','1' little-endian
static constexpr uint32_t VEG_VERSION = 1;
static constexpr size_t HEADER_SIZE = 32;
static constexpr size_t RECORD_SIZE = 24;

// ── Constructor / Destructor ────────────────────────────────────────────────

S3DVegetationManager::S3DVegetationManager() = default;

S3DVegetationManager::~S3DVegetationManager()
{
	stop_worker();
	tiles.clear();
}

// ── Worker threads ──────────────────────────────────────────────────────────

void S3DVegetationManager::start_worker()
{
	if (worker_running.load()) return;
	worker_running.store(true);
	for (int i = 0; i < NUM_WORKERS; i++) {
		worker_threads.emplace_back(&S3DVegetationManager::worker_func, this);
	}
}

void S3DVegetationManager::stop_worker()
{
	worker_running.store(false);
	work_cv.notify_all();
	for (auto &t : worker_threads) {
		if (t.joinable()) t.join();
	}
	worker_threads.clear();
}

static uint32_t s3d_read_u32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
		 | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static double s3d_read_f64(const uint8_t *p) {
	double v;
	std::memcpy(&v, p, 8);
	return v;
}

static float s3d_read_f32(const uint8_t *p) {
	float v;
	std::memcpy(&v, p, 4);
	return v;
}

void S3DVegetationManager::worker_func()
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

		std::ifstream f(req.path, std::ios::binary | std::ios::ate);
		if (!f.is_open()) {
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}
		std::streamsize size = f.tellg();
		f.seekg(0, std::ios::beg);
		std::vector<uint8_t> buf((size_t)size);
		f.read(reinterpret_cast<char *>(buf.data()), size);
		f.close();

		if (buf.size() < HEADER_SIZE) {
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}
		if (s3d_read_u32(&buf[0]) != VEG_MAGIC ||
		    s3d_read_u32(&buf[4]) != VEG_VERSION) {
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}
		uint32_t count = s3d_read_u32(&buf[8]);
		result.tile_east = s3d_read_f64(&buf[16]);
		result.tile_north = s3d_read_f64(&buf[24]);

		size_t expected = HEADER_SIZE + (size_t)count * RECORD_SIZE;
		if (buf.size() < expected) {
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}

		result.trees.reserve(count);
		const uint8_t *p = &buf[HEADER_SIZE];
		for (uint32_t i = 0; i < count; i++) {
			TreeInstance t;
			t.dx              = s3d_read_f32(p + 0);
			t.dz              = s3d_read_f32(p + 4);
			t.ground_z        = s3d_read_f32(p + 8);
			t.height_m        = s3d_read_f32(p + 12);
			t.yaw_rad         = s3d_read_f32(p + 16);
			t.crown_radius_m  = s3d_read_f32(p + 20);
			result.trees.push_back(t);
			p += RECORD_SIZE;
		}
		result.success = true;

		{
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
		}
	}
}

// ── Manifest discovery ──────────────────────────────────────────────────────

void S3DVegetationManager::load_manifests()
{
	manifests.clear();
	for (int i = 0; i < vegetation_paths.size(); i++) {
		String dir = vegetation_paths[i];
		std::vector<ManifestEntry> group;

		Ref<DirAccess> da = DirAccess::open(dir);
		if (da.is_null()) {
			UtilityFunctions::printerr("S3DVegetationManager: cannot open dir ", dir);
			manifests.push_back(group);
			continue;
		}
		da->list_dir_begin();
		String name;
		while (true) {
			name = da->get_next();
			if (name.is_empty()) break;
			if (da->current_is_dir()) continue;
			if (!name.begins_with("vegetation_") || !name.ends_with(".bin")) continue;
			// vegetation_{E}_{N}.bin
			String stem = name.substr(0, name.length() - 4);
			PackedStringArray parts = stem.split("_");
			if (parts.size() < 3) continue;
			ManifestEntry e;
			e.path = String(dir + "/" + name).utf8().get_data();
			e.tile_e = parts[parts.size() - 2].to_int();
			e.tile_n = parts[parts.size() - 1].to_int();
			group.push_back(e);
		}
		da->list_dir_end();
		manifests.push_back(group);
	}
	manifest_loaded = true;
}

// ── Tile streaming ──────────────────────────────────────────────────────────

void S3DVegetationManager::update_tiles(Vector3 camera_pos)
{
	double cam_e = origin_east - (double)camera_pos.x;
	double cam_n = (double)camera_pos.z + origin_north;
	last_cam_e = cam_e;
	last_cam_n = cam_n;

	int load_r2 = load_radius_m * load_radius_m;
	int unload_r = load_radius_m + unload_margin_m;
	int unload_r2 = unload_r * unload_r;

	std::unordered_set<std::string> wanted;

	for (size_t gi = 0; gi < manifests.size(); gi++) {
		for (const auto &e : manifests[gi]) {
			double cx = e.tile_e + 512.0;
			double cy = e.tile_n + 512.0;
			double dx = cx - cam_e;
			double dy = cy - cam_n;
			double d2 = dx * dx + dy * dy;
			if (d2 > load_r2) continue;

			char key[64];
			std::snprintf(key, sizeof(key), "%zu:%d_%d", gi, e.tile_e, e.tile_n);
			std::string id(key);
			wanted.insert(id);

			auto it = tiles.find(id);
			if (it == tiles.end()) {
				TileState st;
				tiles[id] = st;
				LoadRequest req;
				req.tile_id = id;
				req.path = e.path;
				{
					std::lock_guard<std::mutex> lock(work_mutex);
					work_queue.push_back(req);
				}
				work_cv.notify_one();
				tiles[id].loading = true;
			}
		}
	}

	// Unload distant tiles.
	for (auto it = tiles.begin(); it != tiles.end();) {
		const std::string &id = it->first;
		auto colon = id.find(':');
		auto under = id.find('_', colon);
		if (colon == std::string::npos || under == std::string::npos) {
			++it; continue;
		}
		int te = std::stoi(id.substr(colon + 1, under - colon - 1));
		int tn = std::stoi(id.substr(under + 1));
		double cx = te + 512.0;
		double cy = tn + 512.0;
		double dx = cx - cam_e;
		double dy = cy - cam_n;
		if (dx * dx + dy * dy > unload_r2) {
			if (it->second.node) {
				it->second.node->queue_free();
			}
			it = tiles.erase(it);
		} else {
			++it;
		}
	}
}

void S3DVegetationManager::process_load_results()
{
	while (true) {
		LoadResult result;
		{
			std::lock_guard<std::mutex> lock(results_mutex);
			if (results_queue.empty()) return;
			result = std::move(results_queue.front());
			results_queue.pop_front();
		}

		auto it = tiles.find(result.tile_id);
		if (it == tiles.end()) continue;

		TileState &st = it->second;
		st.loading = false;
		st.loaded = true;

		if (!result.success || result.trees.empty() || tree_mesh.is_null()) {
			st.no_data = true;
			continue;
		}

		Ref<MultiMesh> mm;
		mm.instantiate();
		mm->set_transform_format(MultiMesh::TRANSFORM_3D);
		mm->set_use_colors(false);
		mm->set_use_custom_data(false);
		mm->set_mesh(tree_mesh);
		mm->set_instance_count((int)result.trees.size());

		double dx_world = origin_east - result.tile_east;
		double dz_world = result.tile_north - origin_north;

		for (size_t i = 0; i < result.trees.size(); i++) {
			const TreeInstance &t = result.trees[i];
			// World position. Note coordinate convention used by other
			// managers: world_x = origin_east - east, world_z = north - origin_north.
			// We place the tile root at (dx_world, 0, dz_world) and emit
			// per-instance transforms relative to that root.
			float wx = -t.dx;          // east_offset is east-from-west; world.x = -east_offset
			float wz = t.dz;           // north_offset → world.z = +offset
			float wy = t.ground_z;

			float scale_y = (float)(t.height_m * height_to_mesh_unit);
			// Crown radius drives XZ scale; fall back to a 1:3 ratio.
			float scale_xz = std::max(t.crown_radius_m, t.height_m * 0.25f)
			                 * (float)height_to_mesh_unit;

			// Light per-instance jitter so identical heights don't look uniform.
			float j = 1.0f + ((float)((i * 2654435761u) & 0xFFFF) / 65535.0f - 0.5f)
			               * 2.0f * (float)scale_jitter;
			scale_y *= j;
			scale_xz *= j;

			Basis b;
			b = b.rotated(Vector3(0, 1, 0), t.yaw_rad);
			b = b.scaled(Vector3(scale_xz, scale_y, scale_xz));
			Transform3D xf(b, Vector3(wx, wy, wz));
			mm->set_instance_transform((int)i, xf);
		}

		MultiMeshInstance3D *mmi = memnew(MultiMeshInstance3D);
		mmi->set_name(String("vegetation_") + String(result.tile_id.c_str()));
		mmi->set_multimesh(mm);
		mmi->set_position(Vector3((float)dx_world, 0.0f, (float)dz_world));
		// Disable tree shadow casting in the demo. Shadow receiving is handled on
		// the foliage materials so terrain shadows do not over-darken the forest.
		mmi->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
		add_child(mmi);
		st.node = mmi;
	}
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

void S3DVegetationManager::_process(double /*delta*/)
{
	if (!manifest_loaded) {
		load_manifests();
		start_worker();
	}

	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (cam) {
		update_tiles(cam->get_global_position());
	}
	process_load_results();
}

// ── Bind / setters ──────────────────────────────────────────────────────────

void S3DVegetationManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_vegetation_paths", "paths"),
	                     &S3DVegetationManager::set_vegetation_paths);
	ClassDB::bind_method(D_METHOD("get_vegetation_paths"),
	                     &S3DVegetationManager::get_vegetation_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "vegetation_paths"),
	             "set_vegetation_paths", "get_vegetation_paths");

	ClassDB::bind_method(D_METHOD("set_tree_mesh", "mesh"),
	                     &S3DVegetationManager::set_tree_mesh);
	ClassDB::bind_method(D_METHOD("get_tree_mesh"),
	                     &S3DVegetationManager::get_tree_mesh);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "tree_mesh",
	                          PROPERTY_HINT_RESOURCE_TYPE, "Mesh"),
	             "set_tree_mesh", "get_tree_mesh");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"),
	                     &S3DVegetationManager::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"),
	                     &S3DVegetationManager::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"),
	             "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"),
	                     &S3DVegetationManager::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"),
	                     &S3DVegetationManager::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"),
	             "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_load_radius_m", "radius"),
	                     &S3DVegetationManager::set_load_radius_m);
	ClassDB::bind_method(D_METHOD("get_load_radius_m"),
	                     &S3DVegetationManager::get_load_radius_m);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius_m"),
	             "set_load_radius_m", "get_load_radius_m");

	ClassDB::bind_method(D_METHOD("set_scale_jitter", "v"),
	                     &S3DVegetationManager::set_scale_jitter);
	ClassDB::bind_method(D_METHOD("get_scale_jitter"),
	                     &S3DVegetationManager::get_scale_jitter);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "scale_jitter"),
	             "set_scale_jitter", "get_scale_jitter");

	ClassDB::bind_method(D_METHOD("set_height_to_mesh_unit", "v"),
	                     &S3DVegetationManager::set_height_to_mesh_unit);
	ClassDB::bind_method(D_METHOD("get_height_to_mesh_unit"),
	                     &S3DVegetationManager::get_height_to_mesh_unit);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "height_to_mesh_unit"),
	             "set_height_to_mesh_unit", "get_height_to_mesh_unit");

	ClassDB::bind_method(D_METHOD("get_active_tile_count"),
	                     &S3DVegetationManager::get_active_tile_count);
}

void S3DVegetationManager::set_vegetation_paths(const PackedStringArray &p_paths)
{
	vegetation_paths = p_paths;
	manifest_loaded = false;
}
PackedStringArray S3DVegetationManager::get_vegetation_paths() const { return vegetation_paths; }

void S3DVegetationManager::set_tree_mesh(const Ref<Mesh> &p_mesh) { tree_mesh = p_mesh; }
Ref<Mesh> S3DVegetationManager::get_tree_mesh() const { return tree_mesh; }

void S3DVegetationManager::set_origin_east(double v) { origin_east = v; }
double S3DVegetationManager::get_origin_east() const { return origin_east; }
void S3DVegetationManager::set_origin_north(double v) { origin_north = v; }
double S3DVegetationManager::get_origin_north() const { return origin_north; }

void S3DVegetationManager::set_load_radius_m(int v) { load_radius_m = std::max(100, v); }
int S3DVegetationManager::get_load_radius_m() const { return load_radius_m; }

void S3DVegetationManager::set_scale_jitter(double v) { scale_jitter = std::clamp(v, 0.0, 0.9); }
double S3DVegetationManager::get_scale_jitter() const { return scale_jitter; }

void S3DVegetationManager::set_height_to_mesh_unit(double v) { height_to_mesh_unit = v; }
double S3DVegetationManager::get_height_to_mesh_unit() const { return height_to_mesh_unit; }

int S3DVegetationManager::get_active_tile_count() const { return (int)tiles.size(); }
