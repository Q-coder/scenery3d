#include "s3d_tile_manager.h"
#include "s3d_tile.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

// --- Constructor / Destructor ---

S3DTileManager::S3DTileManager()
{
	rebuild_lod_rings();

	shared_material.instantiate();
	shared_material->set_albedo(Color(0.45, 0.55, 0.35));
	shared_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
}

S3DTileManager::~S3DTileManager()
{
	stop_worker();
	tiles.clear();
}

// --- LOD ring configuration ---

void S3DTileManager::rebuild_lod_rings()
{
	lod_rings.clear();
	if (load_radius <= 5) {
		lod_rings.push_back({load_radius, 3});
	} else if (load_radius <= 15) {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({load_radius, 4});
	} else if (load_radius <= 50) {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({8, 4});
		lod_rings.push_back({load_radius, 6});
	} else {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({8, 4});
		lod_rings.push_back({25, 6});
		lod_rings.push_back({load_radius, 8});
	}
}

int S3DTileManager::lod_for_distance(int dist) const
{
	for (const auto &ring : lod_rings) {
		if (dist <= ring.radius) {
			return ring.lod_level;
		}
	}
	return lod_rings.back().lod_level;
}

// --- Background worker thread pool ---

void S3DTileManager::start_worker()
{
	if (worker_running.load()) return;
	worker_running.store(true);
	for (int i = 0; i < NUM_WORKERS; i++) {
		worker_threads.emplace_back(&S3DTileManager::worker_func, this);
	}
}

void S3DTileManager::stop_worker()
{
	if (!worker_running.load()) return;
	worker_running.store(false);
	work_cv.notify_all();
	for (auto &t : worker_threads) {
		if (t.joinable()) t.join();
	}
	worker_threads.clear();
}

void S3DTileManager::worker_func()
{
	while (worker_running.load()) {
		LoadRequest req;
		{
			std::unique_lock<std::mutex> lock(work_mutex);
			work_cv.wait(lock, [this] {
				return !work_queue.empty() || !worker_running.load();
			});
			if (!worker_running.load()) break;
			if (work_queue.empty()) continue;
			req = std::move(work_queue.front());
			work_queue.pop_front();
		}

		// Read file using C++ I/O (thread-safe, no Godot API calls).
		LoadResult result;
		result.ei = req.ei;
		result.ni = req.ni;
		result.key = req.key;
		result.tile_size = req.tile_size;
		result.desired_lod = req.desired_lod;
		result.success = false;

		size_t expected_size = (size_t)req.tile_size * req.tile_size * 4;
		std::ifstream file(req.path, std::ios::binary);
		if (file.is_open()) {
			result.raw_bytes.resize(expected_size);
			file.read(reinterpret_cast<char *>(result.raw_bytes.data()), expected_size);
			if ((size_t)file.gcount() == expected_size) {
				result.success = true;
			}
			file.close();
		}

		{
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
		}
	}
}

// --- Process completed file loads on main thread ---

void S3DTileManager::process_load_results(int &verts_generated)
{
	while (true) {
		LoadResult result;
		{
			std::lock_guard<std::mutex> lock(results_mutex);
			if (results_queue.empty()) break;
			result = std::move(results_queue.front());
			results_queue.pop_front();
		}

		auto it = tiles.find(result.key);
		if (it == tiles.end()) {
			// Tile was unloaded while loading — discard result.
			continue;
		}

		TileState &state = it->second;
		state.loading = false;

		if (!result.success) {
			// File not found — mark as no-data so we don't retry.
			state.no_data = true;
			continue;
		}

		// Check vertex budget before meshing.
		int desired_lod = state.desired_lod >= 0 ? state.desired_lod : result.desired_lod;
		int stride = 1 << desired_lod;
		int verts_per_axis = (result.tile_size - 1) / stride + 1;
		int tile_verts = verts_per_axis * verts_per_axis;

		if (verts_generated + tile_verts > VERTEX_BUDGET_PER_FRAME && verts_generated > 0) {
			// Over budget — push result back for next frame.
			state.loading = true;
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_front(std::move(result));
			break;
		}

		// Create Image from raw bytes on main thread.
		PackedByteArray bytes;
		bytes.resize(result.raw_bytes.size());
		memcpy(bytes.ptrw(), result.raw_bytes.data(), result.raw_bytes.size());
		result.raw_bytes.clear();

		Ref<Image> heightmap = Image::create_from_data(
			result.tile_size, result.tile_size, false, Image::FORMAT_RF, bytes);
		if (heightmap.is_null()) {
			state.no_data = true;
			continue;
		}

		// No flips needed: provpilot's conversion already placed
		// pixel(0,0) at SE corner, matching our convention where
		// gx=0,gz=0 maps to (most-East=lowest-X, most-South=lowest-Z).

		// Create or update the tile node.
		if (!state.node) {
			S3DTile *tile = memnew(S3DTile);
			tile->set_tile_x(result.ei);
			tile->set_tile_z(result.ni);
			tile->set_tile_size(result.tile_size);
			tile->set_material(shared_material);

			// +X = West: east edge (lowest X) at origin_E - (ei+1)*tile_size.
			double world_x = origin_east - (double)(result.ei + 1) * tile_size;
			double world_z = (double)result.ni * tile_size - origin_north;
			tile->set_position(Vector3(world_x, 0.0, world_z));

			add_child(tile);
			state.node = tile;
		}

		state.node->set_heightmap(heightmap);
		state.node->set_lod_level(desired_lod);
		state.node->generate_mesh();
		state.current_lod = desired_lod;
		verts_generated += tile_verts;

		// Register with elevation DB for close tiles only.
		if (desired_lod < LOD_DISCARD_THRESHOLD && elevation_db.is_valid()) {
			elevation_db->load_tile(result.ei, result.ni, heightmap);
		}

		// Discard heightmap for distant tiles to save memory.
		if (desired_lod >= LOD_DISCARD_THRESHOLD) {
			state.node->set_heightmap(Ref<Image>());
		}
	}
}

// --- Bind methods ---

void S3DTileManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("update_tiles", "camera_pos"), &S3DTileManager::update_tiles);
	ClassDB::bind_method(D_METHOD("get_active_tile_count"), &S3DTileManager::get_active_tile_count);

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DTileManager::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DTileManager::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_load_radius", "radius"), &S3DTileManager::set_load_radius);
	ClassDB::bind_method(D_METHOD("get_load_radius"), &S3DTileManager::get_load_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius"), "set_load_radius", "get_load_radius");

	ClassDB::bind_method(D_METHOD("set_load_budget", "budget"), &S3DTileManager::set_load_budget);
	ClassDB::bind_method(D_METHOD("get_load_budget"), &S3DTileManager::get_load_budget);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_budget"), "set_load_budget", "get_load_budget");

	ClassDB::bind_method(D_METHOD("set_unload_margin", "margin"), &S3DTileManager::set_unload_margin);
	ClassDB::bind_method(D_METHOD("get_unload_margin"), &S3DTileManager::get_unload_margin);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "unload_margin"), "set_unload_margin", "get_unload_margin");

	ClassDB::bind_method(D_METHOD("set_data_path", "path"), &S3DTileManager::set_data_path);
	ClassDB::bind_method(D_METHOD("get_data_path"), &S3DTileManager::get_data_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "data_path"), "set_data_path", "get_data_path");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DTileManager::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DTileManager::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DTileManager::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DTileManager::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");
}

// --- _process ---

void S3DTileManager::_process(double delta)
{
	Viewport *vp = get_viewport();
	if (!vp) return;

	Camera3D *camera = vp->get_camera_3d();
	if (!camera) return;

	// Start worker threads on first frame.
	if (!worker_running.load()) {
		start_worker();
	}

	update_tiles(camera->get_global_position());
}

// --- Main update logic ---

void S3DTileManager::update_tiles(Vector3 camera_pos)
{
	if (lod_rings.empty()) return;

	// Convert camera world position to LV95 tile indices.
	// +X = West, so LV95_E = origin_east - camera_pos.x
	double cam_lv95_e = origin_east - camera_pos.x;
	double cam_lv95_n = camera_pos.z + origin_north;
	int cam_ei = (int)std::floor(cam_lv95_e / (double)tile_size);
	int cam_ni = (int)std::floor(cam_lv95_n / (double)tile_size);

	int max_radius = lod_rings.back().radius;

	// 1. Compute desired LOD for all tiles in range and track required set.
	std::unordered_set<uint64_t> required;
	std::vector<LoadRequest> new_requests;

	for (int dei = -max_radius; dei <= max_radius; dei++) {
		for (int dni = -max_radius; dni <= max_radius; dni++) {
			int dist = std::max(std::abs(dei), std::abs(dni));
			if (dist > max_radius) continue;

			int ei = cam_ei + dei;
			int ni = cam_ni + dni;
			uint64_t key = tile_key(ei, ni);
			int desired_lod = lod_for_distance(dist);

			required.insert(key);

			auto it = tiles.find(key);
			if (it == tiles.end()) {
				// New tile — create state and queue file load.
				TileState state;
				state.desired_lod = desired_lod;
				state.loading = true;
				tiles[key] = state;

				int lv95_east = ei * tile_size;
				int lv95_north = ni * tile_size;
				String gpath = data_path + "/tile_" + itos(lv95_east) + "_" + itos(lv95_north) + ".raw";

				LoadRequest req;
				req.ei = ei;
				req.ni = ni;
				req.key = key;
				req.path = std::string(gpath.utf8().get_data());
				req.tile_size = tile_size;
				req.desired_lod = desired_lod;
				req.distance = dist;
				new_requests.push_back(std::move(req));
			} else {
				// Existing tile — update desired LOD.
				it->second.desired_lod = desired_lod;
			}
		}
	}

	// 2. Unload tiles outside range.
	std::vector<uint64_t> to_remove;
	for (auto &[key, state] : tiles) {
		if (required.find(key) == required.end()) {
			to_remove.push_back(key);
		}
	}

	for (uint64_t key : to_remove) {
		auto it = tiles.find(key);
		if (it != tiles.end()) {
			TileState &state = it->second;
			if (state.node) {
				if (elevation_db.is_valid()) {
					elevation_db->unload_tile(state.node->get_tile_x(), state.node->get_tile_z());
				}
				state.node->queue_free();
			}
			tiles.erase(it);
		}
	}

	// 3. Process completed file loads from worker thread (budget-limited).
	int verts_generated = 0;
	process_load_results(verts_generated);

	// 4. Handle LOD transitions for existing tiles.
	for (auto &[key, state] : tiles) {
		if (verts_generated >= VERTEX_BUDGET_PER_FRAME) break;

		if (state.node && !state.loading && !state.no_data &&
			state.desired_lod != state.current_lod &&
			state.desired_lod >= 0) {

			// Check if tile has cached heightmap for immediate re-mesh.
			if (state.node->get_heightmap().is_valid()) {
				int stride = 1 << state.desired_lod;
				int verts_per_axis = (tile_size - 1) / stride + 1;
				int tile_verts = verts_per_axis * verts_per_axis;

				state.node->set_lod_level(state.desired_lod);
				state.node->generate_mesh();

				// Update elevation DB registration.
				if (state.current_lod >= LOD_DISCARD_THRESHOLD &&
					state.desired_lod < LOD_DISCARD_THRESHOLD &&
					elevation_db.is_valid()) {
					elevation_db->load_tile(
						state.node->get_tile_x(),
						state.node->get_tile_z(),
						state.node->get_heightmap());
				} else if (state.current_lod < LOD_DISCARD_THRESHOLD &&
						   state.desired_lod >= LOD_DISCARD_THRESHOLD &&
						   elevation_db.is_valid()) {
					elevation_db->unload_tile(
						state.node->get_tile_x(),
						state.node->get_tile_z());
				}

				state.current_lod = state.desired_lod;
				verts_generated += tile_verts;

				// Discard heightmap if now distant.
				if (state.desired_lod >= LOD_DISCARD_THRESHOLD) {
					state.node->set_heightmap(Ref<Image>());
				}
			} else {
				// No cached heightmap — need file re-read for LOD change.
				if (!state.loading) {
					state.loading = true;
					int ei = state.node->get_tile_x();
					int ni = state.node->get_tile_z();
					int lv95_east = ei * tile_size;
					int lv95_north = ni * tile_size;
					String gpath = data_path + "/tile_" + itos(lv95_east) + "_" + itos(lv95_north) + ".raw";

					LoadRequest req;
					req.ei = ei;
					req.ni = ni;
					req.key = key;
					req.path = std::string(gpath.utf8().get_data());
					req.tile_size = tile_size;
					req.desired_lod = state.desired_lod;
					req.distance = 0; // High priority for LOD upgrades.
					new_requests.push_back(std::move(req));
				}
			}
		}
	}

	// 5. Sort new requests by distance (closest first) and queue to worker.
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

// --- Getters / Setters ---

int S3DTileManager::get_active_tile_count() const
{
	return (int)tiles.size();
}

void S3DTileManager::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DTileManager::get_tile_size() const
{
	return tile_size;
}

void S3DTileManager::set_load_radius(int p_radius)
{
	load_radius = p_radius;
	rebuild_lod_rings();
}

int S3DTileManager::get_load_radius() const
{
	return load_radius;
}

void S3DTileManager::set_load_budget(int p_budget)
{
	load_budget = p_budget;
}

int S3DTileManager::get_load_budget() const
{
	return load_budget;
}

void S3DTileManager::set_unload_margin(int p_margin)
{
	unload_margin = p_margin;
}

int S3DTileManager::get_unload_margin() const
{
	return unload_margin;
}

void S3DTileManager::set_data_path(const String &p_path)
{
	data_path = p_path;
}

String S3DTileManager::get_data_path() const
{
	return data_path;
}

void S3DTileManager::set_origin_east(double p_east)
{
	origin_east = p_east;
}

double S3DTileManager::get_origin_east() const
{
	return origin_east;
}

void S3DTileManager::set_origin_north(double p_north)
{
	origin_north = p_north;
}

double S3DTileManager::get_origin_north() const
{
	return origin_north;
}

void S3DTileManager::set_elevation_db(Ref<S3DElevationDB> p_db)
{
	elevation_db = p_db;
}

Ref<S3DElevationDB> S3DTileManager::get_elevation_db() const
{
	return elevation_db;
}
