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
	chunks.clear();
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

		LoadResult result;
		result.ei = req.ei;
		result.ni = req.ni;
		result.key = req.key;
		result.is_chunk = req.is_chunk;
		result.success = false;

		if (req.is_chunk) {
			// --- Chunk: read multiple tile files, composite into small heightmap ---
			int cres = req.composite_res;
			int csize = req.chunk_tile_count;
			int base_ei = req.chunk_grid_ei * csize;
			int base_ni = req.chunk_grid_ni * csize;
			int ts = req.tile_size;

			result.composite_res = cres;
			result.tile_size = csize * ts;
			result.desired_lod = 0;

			std::vector<float> composite(cres * cres, 0.0f);
			bool any_data = false;

			// Cache tile data to avoid re-reading the same file.
			std::unordered_map<uint64_t, std::vector<uint8_t>> tile_cache;
			size_t expected = (size_t)ts * ts * 4;

			for (int cy = 0; cy < cres; cy++) {
				for (int cx = 0; cx < cres; cx++) {
					// Composite pixel (0,0) = SE corner of chunk.
					// cx goes E→W, cy goes S→N (matching provpilot convention).
					double frac_x = (double)cx / (cres - 1); // 0=east edge, 1=west edge
					double frac_y = (double)cy / (cres - 1); // 0=south edge, 1=north edge

					double lv95_e = (double)(base_ei + csize) * ts - frac_x * csize * ts;
					double lv95_n = (double)base_ni * ts + frac_y * csize * ts;

					int tei = (int)std::floor(lv95_e / ts);
					int tni = (int)std::floor(lv95_n / ts);

					// Clamp to chunk bounds.
					if (tei < base_ei) tei = base_ei;
					if (tei >= base_ei + csize) tei = base_ei + csize - 1;
					if (tni < base_ni) tni = base_ni;
					if (tni >= base_ni + csize) tni = base_ni + csize - 1;

					uint64_t tkey = tile_key(tei, tni);
					auto cache_it = tile_cache.find(tkey);
					if (cache_it == tile_cache.end()) {
						int lv95_east = tei * ts;
						int lv95_north = tni * ts;
						std::string tpath = req.base_path + "/tile_"
							+ std::to_string(lv95_east) + "_"
							+ std::to_string(lv95_north) + ".raw";
						std::vector<uint8_t> data;
						std::ifstream file(tpath, std::ios::binary);
						if (file.is_open()) {
							data.resize(expected);
							file.read(reinterpret_cast<char *>(data.data()), expected);
							if ((size_t)file.gcount() != expected) data.clear();
							file.close();
						}
						tile_cache[tkey] = std::move(data);
						cache_it = tile_cache.find(tkey);
					}

					if (cache_it->second.empty()) continue;

					// Sample pixel from raw tile.
					// Raw: pixel(0,0) = SE, col goes E→W, row goes S→N.
					double local_e = lv95_e - (double)tei * ts;
					double local_n = lv95_n - (double)tni * ts;

					// col 0 = east edge (local_e = ts), col ts-1 = west edge (local_e = 0)
					int pcol = (int)std::round((ts - local_e) / ts * (ts - 1));
					// row 0 = south edge (local_n = 0), row ts-1 = north edge (local_n = ts)
					int prow = (int)std::round(local_n / ts * (ts - 1));

					pcol = std::max(0, std::min(ts - 1, pcol));
					prow = std::max(0, std::min(ts - 1, prow));

					size_t offset = ((size_t)prow * ts + pcol) * 4;
					float height;
					memcpy(&height, &cache_it->second[offset], 4);

					composite[cy * cres + cx] = height;
					any_data = true;
				}
			}

			if (any_data) {
				result.raw_bytes.resize(cres * cres * 4);
				memcpy(result.raw_bytes.data(), composite.data(), cres * cres * 4);
				result.success = true;
			}
		} else {
			// --- Regular tile: read single file ---
			result.tile_size = req.tile_size;
			result.desired_lod = req.desired_lod;

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

		if (result.is_chunk) {
			// --- Process chunk result ---
			auto it = chunks.find(result.key);
			if (it == chunks.end()) continue;

			TileState &state = it->second;
			state.loading = false;

			if (!result.success) {
				state.no_data = true;
				continue;
			}

			int cres = result.composite_res;
			int chunk_verts = cres * cres;
			if (verts_generated + chunk_verts > VERTEX_BUDGET_PER_FRAME && verts_generated > 0) {
				state.loading = true;
				std::lock_guard<std::mutex> lock(results_mutex);
				results_queue.push_front(std::move(result));
				break;
			}

			PackedByteArray bytes;
			bytes.resize(result.raw_bytes.size());
			memcpy(bytes.ptrw(), result.raw_bytes.data(), result.raw_bytes.size());

			Ref<Image> heightmap = Image::create_from_data(
				cres, cres, false, Image::FORMAT_RF, bytes);
			if (heightmap.is_null()) {
				state.no_data = true;
				continue;
			}

			if (!state.node) {
				S3DTile *tile = memnew(S3DTile);
				tile->set_tile_x(result.ei);
				tile->set_tile_z(result.ni);
				// Chunk covers chunk_size * tile_size meters, not just tile_size.
				tile->set_tile_size(tile_size * chunk_size);
				tile->set_material(shared_material);

				int base_ei = result.ei * chunk_size;
				int base_ni = result.ni * chunk_size;
				double world_x = origin_east - (double)(base_ei + chunk_size) * tile_size;
				double world_z = (double)base_ni * tile_size - origin_north;
				tile->set_position(Vector3(world_x, 0.0, world_z));

				add_child(tile);
				state.node = tile;
			}

			state.node->set_heightmap(heightmap);
			state.node->set_lod_level(0); // Use every pixel.
			state.node->generate_mesh();
			state.current_lod = 0;
			state.node->set_heightmap(Ref<Image>()); // Free memory.
			verts_generated += chunk_verts;
			continue;
		}

		// --- Process regular tile result ---
		auto it = tiles.find(result.key);
		if (it == tiles.end()) {
			continue;
		}

		TileState &state = it->second;
		state.loading = false;

		if (!result.success) {
			state.no_data = true;
			continue;
		}

		int desired_lod = state.desired_lod >= 0 ? state.desired_lod : result.desired_lod;
		int stride = 1 << desired_lod;
		int verts_per_axis = (result.tile_size - 1) / stride + 1;
		int tile_verts = verts_per_axis * verts_per_axis;

		if (verts_generated + tile_verts > VERTEX_BUDGET_PER_FRAME && verts_generated > 0) {
			state.loading = true;
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_front(std::move(result));
			break;
		}

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

		if (!state.node) {
			S3DTile *tile = memnew(S3DTile);
			tile->set_tile_x(result.ei);
			tile->set_tile_z(result.ni);
			tile->set_tile_size(result.tile_size);
			tile->set_material(shared_material);

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

		if (desired_lod < LOD_DISCARD_THRESHOLD && elevation_db.is_valid()) {
			elevation_db->load_tile(result.ei, result.ni, heightmap);
		}

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

	ClassDB::bind_method(D_METHOD("set_far_radius", "radius"), &S3DTileManager::set_far_radius);
	ClassDB::bind_method(D_METHOD("get_far_radius"), &S3DTileManager::get_far_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "far_radius"), "set_far_radius", "get_far_radius");

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

	// 2. Unload tiles outside range (with hysteresis margin).
	// For tiles entering the chunk zone, only remove if the covering chunk
	// is loaded so there's no gap during the handoff.
	int unload_radius = max_radius + unload_margin;
	bool have_chunks = (far_radius > max_radius && chunk_size >= 1);
	std::vector<uint64_t> to_remove;
	for (auto &[key, state] : tiles) {
		int ei = (int)(int32_t)(key >> 32);
		int ni = (int)(int32_t)(key & 0xFFFFFFFF);
		int dist = std::max(std::abs(ei - cam_ei), std::abs(ni - cam_ni));
		if (dist > unload_radius) {
			if (have_chunks && dist <= far_radius) {
				// Tile is in chunk zone — only unload if chunk is ready.
				int cg_ei = (ei >= 0) ? ei / chunk_size : (ei - chunk_size + 1) / chunk_size;
				int cg_ni = (ni >= 0) ? ni / chunk_size : (ni - chunk_size + 1) / chunk_size;
				uint64_t ckey = tile_key(cg_ei, cg_ni);
				auto cit = chunks.find(ckey);
				if (cit != chunks.end() && !cit->second.loading && cit->second.node) {
					to_remove.push_back(key);
				}
				// else: keep tile until chunk is ready
			} else {
				to_remove.push_back(key);
			}
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

	// ===== FAR TERRAIN CHUNKS =====
	if (far_radius <= max_radius || chunk_size < 1) return;

	int far_chunk_radius = (far_radius + chunk_size - 1) / chunk_size;
	int cam_cg_ei = (cam_ei >= 0)
		? cam_ei / chunk_size
		: (cam_ei - chunk_size + 1) / chunk_size;
	int cam_cg_ni = (cam_ni >= 0)
		? cam_ni / chunk_size
		: (cam_ni - chunk_size + 1) / chunk_size;

	std::unordered_set<uint64_t> required_chunks;
	std::vector<LoadRequest> chunk_requests;
	std::string base_path_str;
	if (!data_path.is_empty()) {
		base_path_str = std::string(data_path.utf8().get_data());
	}

	for (int dcg_ei = -far_chunk_radius; dcg_ei <= far_chunk_radius; dcg_ei++) {
		for (int dcg_ni = -far_chunk_radius; dcg_ni <= far_chunk_radius; dcg_ni++) {
			int cg_ei = cam_cg_ei + dcg_ei;
			int cg_ni = cam_cg_ni + dcg_ni;

			// Compute tile distances from this chunk to camera.
			int base_ei = cg_ei * chunk_size;
			int base_ni = cg_ni * chunk_size;
			int closest_ei = std::max(base_ei, std::min(cam_ei, base_ei + chunk_size - 1));
			int closest_ni = std::max(base_ni, std::min(cam_ni, base_ni + chunk_size - 1));
			int min_dist = std::max(std::abs(closest_ei - cam_ei), std::abs(closest_ni - cam_ni));

			// Max distance: farthest tile in chunk from camera.
			int max_dei = std::max(std::abs(base_ei - cam_ei), std::abs(base_ei + chunk_size - 1 - cam_ei));
			int max_dni = std::max(std::abs(base_ni - cam_ni), std::abs(base_ni + chunk_size - 1 - cam_ni));
			int max_dist = std::max(max_dei, max_dni);

			// Only skip if ALL tiles in chunk are covered by individual tiles.
			if (max_dist <= max_radius) continue;
			// Skip chunks beyond far radius (use closest tile).
			if (min_dist > far_radius) continue;

			uint64_t ckey = tile_key(cg_ei, cg_ni);
			required_chunks.insert(ckey);

			auto it = chunks.find(ckey);
			if (it == chunks.end()) {
				TileState state;
				state.desired_lod = 0;
				state.loading = true;
				chunks[ckey] = state;

				LoadRequest req;
				req.is_chunk = true;
				req.ei = cg_ei;
				req.ni = cg_ni;
				req.key = ckey;
				req.chunk_grid_ei = cg_ei;
				req.chunk_grid_ni = cg_ni;
				req.chunk_tile_count = chunk_size;
				req.composite_res = CHUNK_COMPOSITE_RES;
				req.tile_size = tile_size;
				req.base_path = base_path_str;
				req.distance = min_dist;
				chunk_requests.push_back(std::move(req));
			}
		}
	}

	// Unload out-of-range chunks (with hysteresis margin).
	// For chunks entering the individual-tile zone, only remove once the
	// individual tiles covering them are actually loaded (no gaps).
	int chunk_unload_far = far_radius + unload_margin;
	std::vector<uint64_t> chunks_to_remove;
	for (auto &[key, state] : chunks) {
		int cg_ei = (int)(int32_t)(key >> 32);
		int cg_ni = (int)(int32_t)(key & 0xFFFFFFFF);
		int base_ei = cg_ei * chunk_size;
		int base_ni = cg_ni * chunk_size;
		int closest_ei = std::max(base_ei, std::min(cam_ei, base_ei + chunk_size - 1));
		int closest_ni = std::max(base_ni, std::min(cam_ni, base_ni + chunk_size - 1));
		int min_dist = std::max(std::abs(closest_ei - cam_ei), std::abs(closest_ni - cam_ni));
		// Max distance: farthest tile in chunk from camera.
		int max_dei = std::max(std::abs(base_ei - cam_ei), std::abs(base_ei + chunk_size - 1 - cam_ei));
		int max_dni = std::max(std::abs(base_ni - cam_ni), std::abs(base_ni + chunk_size - 1 - cam_ni));
		int max_dist_in_chunk = std::max(max_dei, max_dni);
		if (min_dist > chunk_unload_far) {
			chunks_to_remove.push_back(key);
		} else if (max_dist_in_chunk <= max_radius) {
			// Chunk fully inside individual tile zone — only remove if all
			// overlapping tile positions are loaded (not still loading).
			bool all_covered = true;
			for (int dei = 0; dei < chunk_size && all_covered; dei++) {
				for (int dni = 0; dni < chunk_size && all_covered; dni++) {
					int tei = base_ei + dei;
					int tni = base_ni + dni;
					int tdist = std::max(std::abs(tei - cam_ei), std::abs(tni - cam_ni));
					if (tdist <= max_radius) {
						uint64_t tkey = tile_key(tei, tni);
						auto it = tiles.find(tkey);
						if (it == tiles.end() || it->second.loading || !it->second.node) {
							all_covered = false;
						}
					}
				}
			}
			if (all_covered) {
				chunks_to_remove.push_back(key);
			}
		}
	}
	for (uint64_t key : chunks_to_remove) {
		auto it = chunks.find(key);
		if (it != chunks.end()) {
			if (it->second.node) {
				it->second.node->queue_free();
			}
			chunks.erase(it);
		}
	}

	// Queue chunk requests (low priority — after individual tiles).
	if (!chunk_requests.empty()) {
		std::sort(chunk_requests.begin(), chunk_requests.end(),
			[](const LoadRequest &a, const LoadRequest &b) {
				return a.distance < b.distance;
			});

		std::lock_guard<std::mutex> lock(work_mutex);
		for (auto &req : chunk_requests) {
			work_queue.push_back(std::move(req));
		}
		work_cv.notify_all();
	}
}

// --- Getters / Setters ---

int S3DTileManager::get_active_tile_count() const
{
	return (int)tiles.size() + (int)chunks.size();
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

void S3DTileManager::set_far_radius(int p_radius)
{
	far_radius = p_radius;
}

int S3DTileManager::get_far_radius() const
{
	return far_radius;
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
