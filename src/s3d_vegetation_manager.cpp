#include "s3d_vegetation_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/geometry_instance3d.hpp>
#include <godot_cpp/classes/json.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <filesystem>
#include <chrono>
#include <godot_cpp/classes/project_settings.hpp>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <fstream>

using namespace godot;

namespace s3d {

// File format constants (must match tools/extract_tlm_forests.py).
static constexpr uint32_t VEG_MAGIC = 0x31474556; // 'V','E','G','1' little-endian
static constexpr uint32_t VEG_VERSION = 1;
static constexpr size_t HEADER_SIZE = 32;
static constexpr size_t RECORD_SIZE = 24;

// ── Constructor / Destructor ────────────────────────────────────────────────

S3DVegetationManager::S3DVegetationManager() = default;

S3DVegetationManager::~S3DVegetationManager()
{
	if (manifest_thread.joinable()) manifest_thread.join();
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
		if (req.forest) {
			// polygons filled with trees here on the worker; corridors below
			std::vector<ForestPolygon> polys;
			std::vector<Corridor> corridors;
			if (parse_forest_file(req.path, polys, corridors, result.tile_east, result.tile_north)) {
				if (!req.trees_disabled) {
					fill_polygons(polys, (uint32_t)std::hash<std::string>{}(req.tile_id), result.trees);
				} else {
					result.forest_fill_skipped = true;
				}
				for (const std::string &cp : req.corridor_paths) {
					if (cp == req.path) continue;
					std::vector<ForestPolygon> dummy;
					double e2, n2;
					parse_forest_file(cp, dummy, corridors, e2, n2);
				}
				apply_corridors(corridors, result.trees);
				result.success = true;
			}
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			continue;
		}

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
		if (!req.corridor_paths.empty()) {
			// roads clear inside the surveyed forests too
			std::vector<Corridor> corridors;
			for (const std::string &cp : req.corridor_paths) {
				std::vector<ForestPolygon> dummy;
				double e2, n2;
				parse_forest_file(cp, dummy, corridors, e2, n2);
			}
			apply_corridors(corridors, result.trees);
		}
		result.success = true;

		{
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
		}
	}
}

// ── Manifest discovery ──────────────────────────────────────────────────────

// The directory scan, off the main thread (std::filesystem: no Godot
// objects touched). vegetation_{E}_{N}.bin / forest_{E}_{N}.bin only.
void S3DVegetationManager::scan_manifests(const std::vector<std::string> &dirs, std::vector<std::vector<ManifestEntry>> &out)
{
	out.clear();
	for (const std::string &dir : dirs) {
		std::vector<ManifestEntry> group;
		std::error_code ec;
		// index.txt (written by the bake: "E N f|v" per tile) spares the listing
		// of a region's hundreds of thousands of files
		{
			std::ifstream idx(dir + "/index.txt");
			if (idx.is_open()) {
				std::string line;
				while (std::getline(idx, line)) {
					int te = 0, tn = 0;
					char kind = 0;
					if (std::sscanf(line.c_str(), "%d %d %c", &te, &tn, &kind) != 3) continue;
					ManifestEntry e;
					e.forest = kind == 'f';
					e.path = dir + "/" + (e.forest ? "forest_" : "vegetation_") + std::to_string(te) + "_" + std::to_string(tn) + ".bin";
					e.tile_e = te;
					e.tile_n = tn;
					group.push_back(e);
				}
				out.push_back(group);
				continue;
			}
		}
		std::filesystem::directory_iterator it(dir, ec);
		if (ec) {
			out.push_back(group);
			continue;
		}
		for (const auto &entry : it) {
			if (!entry.is_regular_file(ec)) continue;
			const std::string name = entry.path().filename().string();
			const bool is_forest = name.rfind("forest_", 0) == 0;
			const bool is_veg = name.rfind("vegetation_", 0) == 0;
			if ((!is_veg && !is_forest) || name.size() < 5 || name.compare(name.size() - 4, 4, ".bin") != 0) continue;
			const size_t us2 = name.rfind('_');
			const size_t us1 = us2 == std::string::npos ? std::string::npos : name.rfind('_', us2 - 1);
			if (us1 == std::string::npos) continue;
			ManifestEntry e;
			e.path = entry.path().string();
			e.tile_e = std::atoi(name.substr(us1 + 1, us2 - us1 - 1).c_str());
			e.tile_n = std::atoi(name.substr(us2 + 1, name.size() - 4 - us2 - 1).c_str());
			e.forest = is_forest;
			group.push_back(e);
		}
		out.push_back(group);
	}
}

// Corridor paths, stand-downs and the square -> (group, index) grid, on the
// scan thread.
void S3DVegetationManager::derive_manifests(std::vector<std::vector<ManifestEntry>> &groups, std::unordered_map<uint64_t, std::vector<std::pair<int, int>>> &grid_out)
{
	// Per square: every forest file's corridors apply to every entry of the
	// square, and a polygon fill stands down where a surveyed tree tile exists.
	std::unordered_map<uint64_t, std::vector<std::string>> forest_paths;
	std::unordered_set<uint64_t> surveyed;
	for (const auto &group : groups) {
		for (const auto &e : group) {
			if (e.forest) forest_paths[square_key(e.tile_e, e.tile_n)].push_back(e.path);
			else surveyed.insert(square_key(e.tile_e, e.tile_n));
		}
	}
	// The same square's forest file in several packs (a pack and its build
	// dir both in the library) fills once: the first group, the rest only
	// contribute corridors.
	std::unordered_set<uint64_t> filled;
	grid_out.clear();
	for (size_t gi = 0; gi < groups.size(); gi++) {
		for (size_t i = 0; i < groups[gi].size(); i++) {
			ManifestEntry &e = groups[gi][i];
			const uint64_t k = square_key(e.tile_e, e.tile_n);
			auto it = forest_paths.find(k);
			if (it != forest_paths.end()) e.corridor_paths = it->second;
			if (e.forest && (surveyed.count(k) || filled.count(k))) e.trees_disabled = true;
			if (e.forest && !e.trees_disabled) filled.insert(k);
			grid_out[k].push_back({(int)gi, (int)i});
		}
	}
}

int S3DVegetationManager::get_manifest_entry_count() const
{
	int n = 0;
	for (const auto &g : manifests) n += (int)g.size();
	return n;
}

// Called on the main thread once a scan has landed.
void S3DVegetationManager::load_manifests()
{
	manifest_loaded = true;
	UtilityFunctions::print("S3DVegetationManager: ", get_manifest_entry_count(), " tiles in ", (int)manifests.size(), " dir(s), scanned and indexed in ", (int)manifest_scan_ms, " ms");
}

// ── Tile streaming ──────────────────────────────────────────────────────────

void S3DVegetationManager::update_tiles(Vector3 camera_pos)
{
	const proj::EN cam_grid = frame.world_to_grid(camera_pos);
	double cam_e = cam_grid.e;
	double cam_n = cam_grid.n;
	last_cam_e = cam_e;
	last_cam_n = cam_n;

	const int radius = card_mode ? std::max(load_radius_m, (int)far_range_m + 800) : load_radius_m;
	int load_r2 = radius * radius;
	int unload_r = radius + unload_margin_m;
	int unload_r2 = unload_r * unload_r;

	std::unordered_set<std::string> wanted;
	// Only the squares inside the radius are looked up (grid index); the
	// manifests themselves may hold a million entries across the regions.
	const int e_lo = (int)std::floor((cam_e - radius) / 1024.0) * 1024;
	const int e_hi = (int)std::floor((cam_e + radius) / 1024.0) * 1024;
	const int n_lo = (int)std::floor((cam_n - radius) / 1024.0) * 1024;
	const int n_hi = (int)std::floor((cam_n + radius) / 1024.0) * 1024;
	for (int te = e_lo; te <= e_hi; te += 1024) {
		for (int tn = n_lo; tn <= n_hi; tn += 1024) {
			auto git = grid.find(square_key(te, tn));
			if (git == grid.end()) continue;
			const double dx = te + 512.0 - cam_e;
			const double dy = tn + 512.0 - cam_n;
			if (dx * dx + dy * dy > load_r2) continue;
			for (const auto &gi_idx : git->second) {
			const size_t gi = (size_t)gi_idx.first;
			const ManifestEntry &e = manifests[gi][(size_t)gi_idx.second];

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
				req.forest = e.forest;
				req.trees_disabled = e.trees_disabled;
				req.corridor_paths = e.corridor_paths;
				{
					std::lock_guard<std::mutex> lock(work_mutex);
					work_queue.push_back(req);
				}
				work_cv.notify_one();
				tiles[id].loading = true;
			}
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
			if (it->second.far_node) {
				it->second.far_node->queue_free();
			}
			it = tiles.erase(it);
		} else {
			++it;
		}
	}
}

// ── Forest polygon tiles (FOR1) ─────────────────────────────────────────────
// header 32 B: "FOR1", u32 version, u32 polygon count, u32 flags, f64 tile
// east, f64 tile north. Polygon: u8 kind (0 broadleaf, 1 conifer, 2 mixed,
// 3 corridor), u8[3] pad, f32 density (corridor: half width m), f32 height
// m, u32 ring count; ring: u32 n, n x (f32 dx, f32 dz). A corridor has one
// "ring": its centre line.
bool S3DVegetationManager::parse_forest_file(const std::string &path, std::vector<ForestPolygon> &polys, std::vector<Corridor> &corridors, double &tile_east, double &tile_north)
{
	return s3d_parse_forest_file(path, polys, corridors, tile_east, tile_north);
}

static inline uint32_t s3d_hash2(uint32_t a, uint32_t b)
{
	uint32_t h = a * 2654435761u ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2));
	h ^= h >> 13; h *= 2246822519u; h ^= h >> 16;
	return h;
}

// A jittered grid at the polygon's spacing (1 / sqrt(density)), points inside
// the outer ring and outside the holes. Deterministic per tile and polygon so
// a reload gives the same forest.
void S3DVegetationManager::fill_polygons(const std::vector<ForestPolygon> &polys, uint32_t seed, std::vector<TreeInstance> &out)
{
	for (size_t pi = 0; pi < polys.size(); pi++) {
		const ForestPolygon &poly = polys[pi];
		if (poly.kind > 2 || poly.density <= 1e-6f) continue;   // land cover (water, fields, towns) gets no trees
		const float spacing = 1.0f / std::sqrt(poly.density);
		const std::vector<float> &outer = poly.rings[0];
		float minx = outer[0], maxx = outer[0], minz = outer[1], maxz = outer[1];
		for (size_t k = 0; k + 1 < outer.size(); k += 2) {
			minx = std::min(minx, outer[k]); maxx = std::max(maxx, outer[k]);
			minz = std::min(minz, outer[k + 1]); maxz = std::max(maxz, outer[k + 1]);
		}
		minx = std::max(minx, 0.0f); minz = std::max(minz, 0.0f);
		maxx = std::min(maxx, 1024.0f); maxz = std::min(maxz, 1024.0f);
		const uint32_t pseed = s3d_hash2(seed, (uint32_t)pi);
		int gi = 0;
		for (float gx = minx; gx < maxx; gx += spacing, gi++) {
			int gj = 0;
			for (float gz = minz; gz < maxz; gz += spacing, gj++) {
				const uint32_t h = s3d_hash2(pseed, (uint32_t)(gi * 7919 + gj));
				const float jx = ((h & 0xFFFF) / 65535.0f - 0.5f) * 0.9f * spacing;
				const float jz = (((h >> 16) & 0xFFFF) / 65535.0f - 0.5f) * 0.9f * spacing;
				const float x = gx + jx, z = gz + jz;
				if (x < 0.0f || z < 0.0f || x >= 1024.0f || z >= 1024.0f) continue;
				if (!s3d_point_in_ring(outer, x, z)) continue;
				bool in_hole = false;
				for (size_t r = 1; r < poly.rings.size() && !in_hole; r++) in_hole = s3d_point_in_ring(poly.rings[r], x, z);
				if (in_hole) continue;
				const uint32_t h2 = s3d_hash2(h, 77u);
				TreeInstance t;
				t.dx = x;
				t.dz = z;
				t.ground_z = 0.0f;   // the build snaps it to the terrain
				t.height_m = poly.height_m * (0.75f + 0.5f * ((h2 & 0xFFFF) / 65535.0f));
				t.yaw_rad = ((h2 >> 16) & 0xFFFF) / 65535.0f * 6.2831853f;
				t.crown_radius_m = 0.0f;
				t.species_hint = poly.kind == 0 ? 0 : poly.kind == 1 ? 1 : -1;
				out.push_back(t);
			}
		}
	}
}

// Drop every tree closer to a corridor's centre line than its half width
// plus a trunk's margin.
void S3DVegetationManager::apply_corridors(const std::vector<Corridor> &corridors, std::vector<TreeInstance> &trees)
{
	if (corridors.empty() || trees.empty()) return;
	std::vector<TreeInstance> kept;
	kept.reserve(trees.size());
	for (const TreeInstance &t : trees) {
		bool blocked = false;
		for (const Corridor &c : corridors) {
			const float r = c.half_width + 1.0f;
			if (t.dx < c.min_x - r || t.dx > c.max_x + r || t.dz < c.min_z - r || t.dz > c.max_z + r) continue;
			for (size_t k = 0; k + 3 < c.xz.size() && !blocked; k += 2) {
				const float ax = c.xz[k], az = c.xz[k + 1], bx = c.xz[k + 2], bz = c.xz[k + 3];
				const float vx = bx - ax, vz = bz - az;
				const float len2 = vx * vx + vz * vz;
				float u = len2 > 1e-6f ? ((t.dx - ax) * vx + (t.dz - az) * vz) / len2 : 0.0f;
				u = std::clamp(u, 0.0f, 1.0f);
				const float px = ax + u * vx - t.dx, pz = az + u * vz - t.dz;
				if (px * px + pz * pz < r * r) blocked = true;
			}
			if (blocked) break;
		}
		if (!blocked) kept.push_back(t);
	}
	trees.swap(kept);
}

// ── Card mode assets ────────────────────────────────────────────────────────
// Three vertical quads at 60 deg: unit height (base at y = 0), unit crown
// width. UV = card space (v = 0 at the top of the card). The shader picks the
// species column.
void S3DVegetationManager::prepare_card_assets()
{
	if (card_assets_ready) return;
	card_assets_ready = true;
	PackedVector3Array verts, normals;
	PackedVector2Array uvs;
	PackedColorArray colors;
	PackedInt32Array idx;
	auto quad = [&](const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &d, float top) {
		const int base = verts.size();
		const Vector3 corners[4] = {a, b, c, d};
		const Vector2 uv[4] = {Vector2(0, 1), Vector2(1, 1), Vector2(1, 0), Vector2(0, 0)};
		for (int i = 0; i < 4; i++) {
			verts.push_back(corners[i]);
			normals.push_back(Vector3(0, 1, 0));
			uvs.push_back(uv[i]);
			colors.push_back(Color(top, 0, 0, 1));
		}
		const int order[6] = {0, 1, 2, 0, 2, 3};
		for (int i = 0; i < 6; i++) idx.push_back(base + order[i]);
	};
	for (int k = 0; k < 3; k++) {
		const double a = k * (3.14159265358979323846 / 3.0); // not M_PI: MSVC <cmath> lacks it without _USE_MATH_DEFINES
		const Vector3 d(0.5 * std::cos(a), 0.0, 0.5 * std::sin(a));
		quad(-d, d, d + Vector3(0, 1, 0), -d + Vector3(0, 1, 0), 0.0f);
	}
	// No canopy quad (gery 2026-11-14): the top-down card fading in on a steep
	// view read as trees popping up while flying over a forest. The shader
	// keeps its COLOR.r branch for a mesh that has one.
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_NORMAL] = normals;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_COLOR] = colors;
	arrays[Mesh::ARRAY_INDEX] = idx;
	Ref<ArrayMesh> am;
	am.instantiate();
	am->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
	am->set_custom_aabb(AABB(Vector3(-0.5, 0, -0.5), Vector3(1, 1, 1)));
	card_mesh = am;

	// species: tree_atlas.json next to the atlas
	species.clear();
	if (FileAccess::file_exists(atlas_meta_path)) {
		Ref<FileAccess> f = FileAccess::open(atlas_meta_path, FileAccess::READ);
		if (f.is_valid()) {
			const Variant parsed = JSON::parse_string(f->get_as_text());
			if (parsed.get_type() == Variant::DICTIONARY) {
				const Array list = Dictionary(parsed).get("species", Array());
				for (int i = 0; i < list.size(); i++) {
					if (list[i].get_type() != Variant::DICTIONARY) continue;
					const Dictionary d = list[i];
					Species sp;
					sp.name = String(d.get("name", "")).utf8().get_data();
					sp.conifer = (bool)d.get("conifer", false);
					sp.height_m = (double)d.get("height_m", 20.0);
					sp.crown_ratio = (double)d.get("crown_ratio", 0.6);
					species.push_back(sp);
				}
			}
		}
	}
	if (species.empty()) {
		UtilityFunctions::push_warning("S3DVegetationManager: no species in ", atlas_meta_path, "; one generic column");
		species.push_back(Species());
	}
	Ref<Shader> sh = ResourceLoader::get_singleton()->load("res://addons/scenery3d/shaders/tree.gdshader");
	Ref<Texture2D> tex;
	if (ResourceLoader::get_singleton()->exists(atlas_path)) {
		tex = ResourceLoader::get_singleton()->load(atlas_path);
	} else {
		UtilityFunctions::push_warning("S3DVegetationManager: tree atlas not found: ", atlas_path);
	}
	card_material.instantiate();
	if (sh.is_valid()) {
		card_material->set_shader(sh);
		card_material->set_shader_parameter("atlas", tex);
		card_material->set_shader_parameter("species_count", (int)species.size());
		card_material->set_shader_parameter("fade_begin_m", far_range_m * 0.8);
		card_material->set_shader_parameter("fade_end_m", far_range_m);
		card_material->set_shader_parameter("lod_split_m", near_range_m);
		card_material->set_shader_parameter("lod_band_m", lod_band_m);
		card_material->set_shader_parameter("wind", std::clamp(wind_speed_mps / 12.0, 0.0, 1.0) * 0.03);
	} else {
		UtilityFunctions::push_warning("S3DVegetationManager: shaders/tree.gdshader not found; cards draw untextured");
	}
}

// Conifers take over with altitude (Swiss treeline mix: beech country below
// 800 m, spruce / larch above 1 400 m); the hash spreads species evenly inside
// each group.
int S3DVegetationManager::pick_species(uint32_t hash, float ground_z, int hint) const
{
	if (species.size() <= 1) return 0;
	const double p_conifer = std::clamp(0.25 + (ground_z - 700.0) / 900.0, 0.2, 0.97);
	const bool conifer = hint == 1 ? true : hint == 0 ? false : ((hash & 0xFFFF) / 65535.0) < p_conifer;
	std::vector<int> pool;
	for (size_t i = 0; i < species.size(); i++) if (species[i].conifer == conifer) pool.push_back((int)i);
	if (pool.empty()) for (size_t i = 0; i < species.size(); i++) pool.push_back((int)i);
	return pool[(hash >> 16) % pool.size()];
}

// One MultiMeshInstance3D for a subset of a tile's records. Card mode: the
// card mesh, scaled to the tree's height and crown, with the species /
// variation in the custom data; mesh mode: the user mesh as before.
MultiMeshInstance3D *S3DVegetationManager::make_instances(const LoadResult &result, const std::vector<size_t> &indices, double scale_mul, const char *suffix, TileState *stats, bool far_set)
{
	if (stats) {
		stats->species_counts.assign(std::max<size_t>(1, species.size()), 0);
		stats->min_height = 1e9f;
		stats->max_height = 0.0f;
		stats->low_conifers = stats->high_conifers = stats->low_total = stats->high_total = 0;
		stats->alpine_total = 0;
		stats->low_height_sum = stats->alpine_height_sum = 0.0f;
		stats->snapped = 0;
		stats->positions.clear();
		stats->ground_min = 1e9f;
		stats->ground_max = -1e9f;
		stats->snap_pending = false;
	}
	const bool snap = card_mode && snap_to_terrain && elevation_db.is_valid();
	// A tile waits for terrain only while its own terrain tile is absent; a
	// tree whose point falls into a neighbouring cell without data keeps the
	// record's height instead of re-queuing the tile at every terrain update.
	if (snap && stats) {
		const int ts = std::max(1, elevation_db->get_tile_size());
		if (!elevation_db->has_any_tile((int)std::floor((result.tile_east + 512.0) / ts), (int)std::floor((result.tile_north + 512.0) / ts))) {
			stats->snap_pending = true;
		}
	}
	Ref<MultiMesh> mm;
	mm.instantiate();
	mm->set_transform_format(MultiMesh::TRANSFORM_3D);
	mm->set_use_colors(false);
	mm->set_use_custom_data(card_mode);
	mm->set_mesh(card_mode ? card_mesh : tree_mesh);
	mm->set_instance_count((int)indices.size());
	const uint32_t tile_hash = (uint32_t)std::hash<std::string>{}(result.tile_id);
	for (size_t k = 0; k < indices.size(); k++) {
		const size_t i = indices[k];
		const TreeInstance &t = result.trees[i];
		// Tree records are flat offsets from the tile's SW corner (anchor);
		// the tile root is placed with its centre on the ellipsoid.
		// world_x = -east_offset, world_z = +north_offset.
		const float wx = -t.dx;
		const float wz = t.dz;
		float wy = t.ground_z;
		if (snap) {
			const double h = elevation_db->get_elevation_grid_any(result.tile_east + (double)t.dx, result.tile_north + (double)t.dz);
			if (!std::isnan(h)) {
				wy = (float)h;
				if (stats) stats->snapped++;
			}
		}
		if (stats) {
			stats->ground_min = std::min(stats->ground_min, wy);
			stats->ground_max = std::max(stats->ground_max, wy);
		}
		uint32_t h = (uint32_t)(i * 2654435761u) ^ tile_hash;
		h ^= h >> 13; h *= 2246822519u; h ^= h >> 16;
		const float j = 1.0f + ((float)(h & 0xFFFF) / 65535.0f - 0.5f) * 2.0f * (float)scale_jitter;
		Basis b;
		float scale_y, scale_xz;
		if (card_mode) {
			const int sp = pick_species(h, t.ground_z, t.species_hint);
			// Trees shrink towards the tree line: full height to 1 300 m, about
			// half at 2 200 m (the records carry one 22 m distribution at every
			// altitude; Swiss spruce stands at 1 900 m are 10-15 m).
			const float alt_f = std::clamp(1.0f - (t.ground_z - 1300.0f) / 1000.0f * 0.55f, 0.45f, 1.0f);
			scale_y = t.height_m * alt_f * j * (float)scale_mul;
			// The atlas card is square and the species' crown width is already in
			// the image (bake framing = max extent), so the card scales uniformly.
			scale_xz = scale_y;
			b = b.rotated(Vector3(0, 1, 0), t.yaw_rad);
			b = b.scaled(Vector3(scale_xz, scale_y, scale_xz));
			mm->set_instance_custom_data((int)k, Color((float)sp, (float)((h >> 8) & 0xFF) / 255.0f, (float)((h >> 20) & 0xFF) / 255.0f, far_set ? 1.0f : 0.0f));
			if (stats) {
				if (debug_keep_positions) { stats->positions.push_back(t.dx); stats->positions.push_back(t.dz); }
				stats->species_counts[sp]++;
				stats->min_height = std::min(stats->min_height, scale_y);
				stats->max_height = std::max(stats->max_height, scale_y);
				const Species &S = species[sp];
				if (t.ground_z < 700.0f) { stats->low_total++; stats->low_height_sum += scale_y; if (S.conifer) stats->low_conifers++; }
				if (t.ground_z > 1300.0f) { stats->high_total++; if (S.conifer) stats->high_conifers++; }
				if (t.ground_z > 1900.0f) { stats->alpine_total++; stats->alpine_height_sum += scale_y; }
			}
		} else {
			scale_y = (float)(t.height_m * height_to_mesh_unit) * j;
			scale_xz = std::max(t.crown_radius_m, t.height_m * 0.25f) * (float)height_to_mesh_unit * j;
			if (mesh_pitch_deg != 0.0) {
				b = b.rotated(Vector3(1, 0, 0), (float)(mesh_pitch_deg * 0.017453292519943295));
			}
			b = b.rotated(Vector3(0, 1, 0), t.yaw_rad);
			b = b.scaled(Vector3(scale_xz, scale_y, scale_xz));
		}
		mm->set_instance_transform((int)k, Transform3D(b, Vector3(wx, wy, wz)));
	}
	MultiMeshInstance3D *mmi = memnew(MultiMeshInstance3D);
	mmi->set_name(String("vegetation_") + String(result.tile_id.c_str()) + suffix);
	mmi->set_multimesh(mm);
	const double half_tile = 512.0;
	mmi->set_transform(frame.place((double)result.tile_east, (double)result.tile_north,
		(double)result.tile_east + half_tile, (double)result.tile_north + half_tile, 0.0));
	// No tree shadows: shadow receiving is handled in the foliage material so
	// terrain shadows do not over-darken the forest.
	mmi->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
	if (card_mode && card_material.is_valid()) {
		mmi->set_material_override(card_material);
	}
	add_child(mmi);
	return mmi;
}

void S3DVegetationManager::process_load_results()
{
	int handled = 0;
	while (handled < std::max(1, results_per_frame)) {
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
		st.forest_fill_skipped = result.forest_fill_skipped;
		if (card_mode) prepare_card_assets();
		if (!result.success || result.trees.empty() || (card_mode ? card_mesh.is_null() : tree_mesh.is_null())) {
			st.no_data = true;
			continue;
		}
		handled++;
		// Apply density_fraction by deterministically selecting a subset of
		// trees per tile. Same trees are kept across density changes for
		// stable visuals. Hash combines tile id with instance index.
		const double frac = std::clamp(density_fraction, 0.0, 1.0);
		std::vector<size_t> keep_indices;
		keep_indices.reserve(result.trees.size());
		if (frac >= 1.0) {
			keep_indices.resize(result.trees.size());
			for (size_t i = 0; i < result.trees.size(); i++) keep_indices[i] = i;
		} else if (frac > 0.0) {
			const uint32_t threshold = (uint32_t)(frac * 4294967295.0);
			const uint32_t tile_hash = (uint32_t)std::hash<std::string>{}(result.tile_id);
			for (size_t i = 0; i < result.trees.size(); i++) {
				uint32_t h = (uint32_t)(i * 2654435761u) ^ tile_hash;
				h ^= h >> 13;
				h *= 2246822519u;
				h ^= h >> 16;
				if (h <= threshold) keep_indices.push_back(i);
			}
		}
		if (keep_indices.empty()) {
			st.no_data = true;
			continue;
		}
		// Exclusion zones of higher-priority packs (an airfield's apron).
		if (!exclusions.empty()) {
			int dir_index = 0;
			const size_t colon = result.tile_id.find(':');
			if (colon != std::string::npos) dir_index = std::atoi(result.tile_id.substr(0, colon).c_str());
			std::vector<size_t> kept;
			kept.reserve(keep_indices.size());
			for (size_t i : keep_indices) {
				const TreeInstance &t = result.trees[i];
				if (exclusions.contains(result.tile_east + (double)t.dx, result.tile_north + (double)t.dz, dir_index)) {
					excluded_count++;
				} else {
					kept.push_back(i);
				}
			}
			keep_indices.swap(kept);
			if (keep_indices.empty()) {
				st.no_data = true;
				continue;
			}
		}
		if (!card_mode) {
			if (st.node) st.node->queue_free();
			st.node = make_instances(result, keep_indices, 1.0, "", nullptr, false);
			st.reloading = false;
			continue;
		}
		// Card mode: the near set (every kept tree) to near_range_m, the far
		// set (every n-th tree, enlarged so the canopy cover stays) beyond. A
		// rebuilt tile (terrain arrived) replaces its nodes in the same frame.
		// The engine's ranges only cull well outside the hand-over band
		// (tile half-diagonal 724 m); the shader crossfades per tree inside it.
		if (st.node) st.node->queue_free();
		if (st.far_node) st.far_node->queue_free();
		st.far_node = nullptr;
		st.reloading = false;
		const float slack = (float)lod_band_m + 800.0f;
		st.node = make_instances(result, keep_indices, 1.0, "_near", &st, false);
		st.node->set_visibility_range_end((float)near_range_m + slack);
		st.node->set_visibility_range_fade_mode(GeometryInstance3D::VISIBILITY_RANGE_FADE_DISABLED);
		const int step = std::max(1, (int)std::lround(1.0 / std::clamp(far_fraction, 0.02, 1.0)));
		std::vector<size_t> far_indices;
		far_indices.reserve(keep_indices.size() / step + 1);
		for (size_t k = 0; k < keep_indices.size(); k += step) far_indices.push_back(keep_indices[k]);
		st.far_node = make_instances(result, far_indices, far_scale, "_far", nullptr, true);
		st.far_height_ratio = (float)far_scale;
		st.far_node->set_visibility_range_begin(std::max(0.0f, (float)near_range_m - slack));
		st.far_node->set_visibility_range_end((float)far_range_m + 1500.0f);
		st.far_node->set_visibility_range_fade_mode(GeometryInstance3D::VISIBILITY_RANGE_FADE_DISABLED);
	}
}

// ── Lifecycle ───────────────────────────────────────────────────────────────

void S3DVegetationManager::_process(double /*delta*/)
{
	if (!manifest_loaded) {
		if (manifest_scanning.load()) {
			if (manifest_ready.load()) {
				if (manifest_thread.joinable()) manifest_thread.join();
				manifest_scanning.store(false);
				manifest_ready.store(false);
				if (manifest_scan_generation == manifest_generation) {
					manifests.swap(manifests_pending);
					manifests_pending.clear();
					grid.swap(grid_pending);
					grid_pending.clear();
					load_manifests();
					start_worker();
				}
				// a stale scan (the paths changed meanwhile) is dropped; the next frame starts a fresh one
			}
		} else {
			std::vector<std::string> dirs;
			for (int i = 0; i < vegetation_paths.size(); i++) {
				String d = vegetation_paths[i];
				if (d.begins_with("res://") || d.begins_with("user://")) d = ProjectSettings::get_singleton()->globalize_path(d);
				dirs.push_back(d.utf8().get_data());
			}
			manifest_scan_generation = manifest_generation;
			manifest_scanning.store(true);
			manifest_ready.store(false);
			manifest_thread = std::thread([this, dirs]() {
				const auto t0 = std::chrono::steady_clock::now();
				scan_manifests(dirs, manifests_pending);
				derive_manifests(manifests_pending, grid_pending);
				manifest_scan_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
				manifest_ready.store(true);
			});
		}
		if (!manifest_loaded) return;
	}

	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (cam) {
		update_tiles(cam->get_global_position());
	}
	process_load_results();
	resnap_pending_tiles();
}

// A tile built before its terrain had arrived took the records' ground
// heights; once the elevation database has that terrain, drop the tile so
// update_tiles rebuilds it on the surface.
void S3DVegetationManager::resnap_pending_tiles()
{
	if (!card_mode || !snap_to_terrain || elevation_db.is_null()) return;
	const uint64_t epoch = elevation_db->get_epoch();
	if (epoch == snap_epoch) return;
	snap_epoch = epoch;
	const int ts = std::max(1, elevation_db->get_tile_size());
	for (auto it = tiles.begin(); it != tiles.end();) {
		TileState &st = it->second;
		if (!st.loaded || !st.snap_pending) { ++it; continue; }
		const std::string &id = it->first;
		const auto colon = id.find(':');
		const auto under = id.find('_', colon);
		if (colon == std::string::npos || under == std::string::npos) { ++it; continue; }
		const int te = std::stoi(id.substr(colon + 1, under - colon - 1));
		const int tn = std::stoi(id.substr(under + 1));
		if (!elevation_db->has_any_tile((int)std::floor((te + 512.0) / ts), (int)std::floor((tn + 512.0) / ts))) { ++it; continue; }
		// rebuild in place: the old nodes stay on screen until the new ones land
		const ManifestEntry *entry = nullptr;
		const int gi = std::atoi(id.substr(0, colon).c_str());
		if (gi >= 0 && gi < (int)manifests.size()) {
			for (const auto &e : manifests[gi]) {
				if (e.tile_e == te && e.tile_n == tn) { entry = &e; break; }
			}
		}
		if (entry == nullptr) { ++it; continue; }
		st.snap_pending = false;
		st.reloading = true;
		{
			LoadRequest req;
			req.tile_id = id;
			req.path = entry->path;
			req.forest = entry->forest;
			req.trees_disabled = entry->trees_disabled;
			req.corridor_paths = entry->corridor_paths;
			std::lock_guard<std::mutex> lock(work_mutex);
			work_queue.push_back(req);
		}
		work_cv.notify_one();
		++it;
	}
}

// ── Bind / setters ──────────────────────────────────────────────────────────

void S3DVegetationManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_exclusion_zones", "zones"), &S3DVegetationManager::set_exclusion_zones);
	ClassDB::bind_method(D_METHOD("get_exclusion_zones"), &S3DVegetationManager::get_exclusion_zones);
	ClassDB::bind_method(D_METHOD("get_excluded_tree_count"), &S3DVegetationManager::get_excluded_tree_count);
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
	ClassDB::bind_method(D_METHOD("set_frame_mode", "mode"), &S3DVegetationManager::set_frame_mode);
	ClassDB::bind_method(D_METHOD("get_frame_mode"), &S3DVegetationManager::get_frame_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "frame_mode"), "set_frame_mode", "get_frame_mode");

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

	ClassDB::bind_method(D_METHOD("set_mesh_pitch_deg", "v"),
	                     &S3DVegetationManager::set_mesh_pitch_deg);
	ClassDB::bind_method(D_METHOD("get_mesh_pitch_deg"),
	                     &S3DVegetationManager::get_mesh_pitch_deg);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mesh_pitch_deg"),
	             "set_mesh_pitch_deg", "get_mesh_pitch_deg");

	ClassDB::bind_method(D_METHOD("set_density_fraction", "v"),
	                     &S3DVegetationManager::set_density_fraction);
	ClassDB::bind_method(D_METHOD("get_density_fraction"),
	                     &S3DVegetationManager::get_density_fraction);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "density_fraction"),
	             "set_density_fraction", "get_density_fraction");

	ClassDB::bind_method(D_METHOD("get_active_tile_count"),
	                     &S3DVegetationManager::get_active_tile_count);
	// card mode
	ClassDB::bind_method(D_METHOD("set_card_mode", "on"), &S3DVegetationManager::set_card_mode);
	ClassDB::bind_method(D_METHOD("get_card_mode"), &S3DVegetationManager::get_card_mode);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "card_mode"), "set_card_mode", "get_card_mode");
	ClassDB::bind_method(D_METHOD("set_atlas_path", "path"), &S3DVegetationManager::set_atlas_path);
	ClassDB::bind_method(D_METHOD("get_atlas_path"), &S3DVegetationManager::get_atlas_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "atlas_path"), "set_atlas_path", "get_atlas_path");
	ClassDB::bind_method(D_METHOD("set_atlas_meta_path", "path"), &S3DVegetationManager::set_atlas_meta_path);
	ClassDB::bind_method(D_METHOD("get_atlas_meta_path"), &S3DVegetationManager::get_atlas_meta_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "atlas_meta_path"), "set_atlas_meta_path", "get_atlas_meta_path");
	ClassDB::bind_method(D_METHOD("set_near_range_m", "v"), &S3DVegetationManager::set_near_range_m);
	ClassDB::bind_method(D_METHOD("get_near_range_m"), &S3DVegetationManager::get_near_range_m);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "near_range_m"), "set_near_range_m", "get_near_range_m");
	ClassDB::bind_method(D_METHOD("set_far_range_m", "v"), &S3DVegetationManager::set_far_range_m);
	ClassDB::bind_method(D_METHOD("get_far_range_m"), &S3DVegetationManager::get_far_range_m);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "far_range_m"), "set_far_range_m", "get_far_range_m");
	ClassDB::bind_method(D_METHOD("set_far_fraction", "v"), &S3DVegetationManager::set_far_fraction);
	ClassDB::bind_method(D_METHOD("get_far_fraction"), &S3DVegetationManager::get_far_fraction);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "far_fraction"), "set_far_fraction", "get_far_fraction");
	ClassDB::bind_method(D_METHOD("set_far_scale", "v"), &S3DVegetationManager::set_far_scale);
	ClassDB::bind_method(D_METHOD("get_far_scale"), &S3DVegetationManager::get_far_scale);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "far_scale"), "set_far_scale", "get_far_scale");
	ClassDB::bind_method(D_METHOD("get_species_count"), &S3DVegetationManager::get_species_count);
	ClassDB::bind_method(D_METHOD("set_elevation_db", "db"), &S3DVegetationManager::set_elevation_db);
	ClassDB::bind_method(D_METHOD("get_elevation_db"), &S3DVegetationManager::get_elevation_db);
	ClassDB::bind_method(D_METHOD("set_snap_to_terrain", "on"), &S3DVegetationManager::set_snap_to_terrain);
	ClassDB::bind_method(D_METHOD("set_wind_speed_mps", "v"), &S3DVegetationManager::set_wind_speed_mps);
	ClassDB::bind_method(D_METHOD("get_wind_speed_mps"), &S3DVegetationManager::get_wind_speed_mps);
	ClassDB::bind_method(D_METHOD("get_snap_to_terrain"), &S3DVegetationManager::get_snap_to_terrain);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "snap_to_terrain"), "set_snap_to_terrain", "get_snap_to_terrain");
	ClassDB::bind_method(D_METHOD("get_card_mesh"), &S3DVegetationManager::get_card_mesh);
	ClassDB::bind_method(D_METHOD("tile_instance_counts", "tile_e", "tile_n"), &S3DVegetationManager::tile_instance_counts);
	ClassDB::bind_method(D_METHOD("tile_stats", "tile_e", "tile_n"), &S3DVegetationManager::tile_stats);
	ClassDB::bind_method(D_METHOD("pending_tile_count"), &S3DVegetationManager::pending_tile_count);
	ClassDB::bind_method(D_METHOD("is_manifest_loaded"), &S3DVegetationManager::is_manifest_loaded);
	ClassDB::bind_method(D_METHOD("get_manifest_entry_count"), &S3DVegetationManager::get_manifest_entry_count);
	ClassDB::bind_method(D_METHOD("get_manifest_scan_ms"), &S3DVegetationManager::get_manifest_scan_ms);
	ClassDB::bind_method(D_METHOD("set_debug_keep_positions", "on"), &S3DVegetationManager::set_debug_keep_positions);
	ClassDB::bind_method(D_METHOD("get_debug_keep_positions"), &S3DVegetationManager::get_debug_keep_positions);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_keep_positions"), "set_debug_keep_positions", "get_debug_keep_positions");
	ClassDB::bind_method(D_METHOD("tile_tree_positions", "tile_e", "tile_n"), &S3DVegetationManager::tile_tree_positions);
}

void S3DVegetationManager::clear_loaded_tiles()
{
	for (auto &[tid, st] : tiles) {
		if (st.node) st.node->queue_free();
		if (st.far_node) st.far_node->queue_free();
	}
	tiles.clear();
	excluded_count = 0;
}

void S3DVegetationManager::set_exclusion_zones(const Array &zones)
{
	exclusion_zones = zones;
	exclusions.set_from_array(zones, "vegetation");
	clear_loaded_tiles();
}

Array S3DVegetationManager::get_exclusion_zones() const { return exclusion_zones; }
int S3DVegetationManager::get_excluded_tree_count() const { return excluded_count; }

void S3DVegetationManager::set_vegetation_paths(const PackedStringArray &p_paths)
{
	vegetation_paths = p_paths;
	manifest_loaded = false;
	manifest_generation++;   // a scan in flight is dropped when it lands
	clear_loaded_tiles();
}
PackedStringArray S3DVegetationManager::get_vegetation_paths() const { return vegetation_paths; }

void S3DVegetationManager::set_tree_mesh(const Ref<Mesh> &p_mesh) { tree_mesh = p_mesh; }
Ref<Mesh> S3DVegetationManager::get_tree_mesh() const { return tree_mesh; }

void S3DVegetationManager::set_origin_east(double v) { origin_east = v; frame.origin_east = v; }
double S3DVegetationManager::get_origin_east() const { return origin_east; }
void S3DVegetationManager::set_origin_north(double v) { origin_north = v; frame.origin_north = v; }
void S3DVegetationManager::set_frame_mode(int p_mode) { frame.mode = (p_mode == S3DFrame::ECEF) ? S3DFrame::ECEF : S3DFrame::FLAT; }
double S3DVegetationManager::get_origin_north() const { return origin_north; }

void S3DVegetationManager::set_load_radius_m(int v) { load_radius_m = std::max(100, v); }
int S3DVegetationManager::get_load_radius_m() const { return load_radius_m; }

void S3DVegetationManager::set_scale_jitter(double v) { scale_jitter = std::clamp(v, 0.0, 0.9); }
double S3DVegetationManager::get_scale_jitter() const { return scale_jitter; }

void S3DVegetationManager::set_height_to_mesh_unit(double v) { height_to_mesh_unit = v; }
double S3DVegetationManager::get_height_to_mesh_unit() const { return height_to_mesh_unit; }

void S3DVegetationManager::set_mesh_pitch_deg(double v) { mesh_pitch_deg = v; }
double S3DVegetationManager::get_mesh_pitch_deg() const { return mesh_pitch_deg; }

void S3DVegetationManager::set_density_fraction(double v) {
	double newv = std::clamp(v, 0.0, 1.0);
	if (std::abs(newv - density_fraction) < 1e-6) return;
	density_fraction = newv;
	// Drop all currently loaded tiles so update_tiles repopulates them
	// using the new density on the next process tick.
	for (auto &kv : tiles) {
		if (kv.second.node) kv.second.node->queue_free();
		if (kv.second.far_node) kv.second.far_node->queue_free();
	}
	tiles.clear();
}
double S3DVegetationManager::get_density_fraction() const { return density_fraction; }

// ── Card mode setters ───────────────────────────────────────────────────────
void S3DVegetationManager::set_card_mode(bool on) { if (on != card_mode) { card_mode = on; card_assets_ready = false; clear_loaded_tiles(); } }
void S3DVegetationManager::set_atlas_path(const String &p) { atlas_path = p; card_assets_ready = false; clear_loaded_tiles(); }
void S3DVegetationManager::set_atlas_meta_path(const String &p) { atlas_meta_path = p; card_assets_ready = false; clear_loaded_tiles(); }
void S3DVegetationManager::set_near_range_m(double v)
{
	near_range_m = std::max(200.0, v);
	if (card_material.is_valid()) card_material->set_shader_parameter("lod_split_m", near_range_m);
	clear_loaded_tiles();
}
void S3DVegetationManager::set_far_range_m(double v)
{
	far_range_m = std::max(near_range_m, v);
	if (card_material.is_valid()) {
		card_material->set_shader_parameter("fade_begin_m", far_range_m * 0.8);
		card_material->set_shader_parameter("fade_end_m", far_range_m);
	}
	clear_loaded_tiles();
}
void S3DVegetationManager::set_far_fraction(double v) { far_fraction = std::clamp(v, 0.02, 1.0); clear_loaded_tiles(); }
void S3DVegetationManager::set_far_scale(double v) { far_scale = std::clamp(v, 1.0, 4.0); clear_loaded_tiles(); }
void S3DVegetationManager::set_wind_speed_mps(double v)
{
	wind_speed_mps = std::max(0.0, v);
	if (card_material.is_valid()) {
		card_material->set_shader_parameter("wind", std::clamp(wind_speed_mps / 12.0, 0.0, 1.0) * 0.03);
	}
}
Ref<Mesh> S3DVegetationManager::get_card_mesh() { prepare_card_assets(); return card_mesh; }
// The square's tile state with trees (several groups may hold the square: a
// forest file that stood down beside a surveyed tile), else the first one.
const S3DVegetationManager::TileState *S3DVegetationManager::find_tile_state(int tile_e, int tile_n) const
{
	char want[64];
	std::snprintf(want, sizeof(want), "%d_%d", tile_e, tile_n);
	const TileState *first = nullptr;
	for (const auto &kv : tiles) {
		const auto colon = kv.first.find(':');
		if (colon == std::string::npos || kv.first.substr(colon + 1) != want) continue;
		if (first == nullptr) first = &kv.second;
		if (kv.second.node && kv.second.node->get_multimesh().is_valid() && kv.second.node->get_multimesh()->get_instance_count() > 0) return &kv.second;
	}
	return first;
}

Dictionary S3DVegetationManager::tile_stats(int tile_e, int tile_n) const
{
	Dictionary out;
	const TileState *found = find_tile_state(tile_e, tile_n);
	if (found) {
		const TileState &st = *found;
		out["near"] = st.node && st.node->get_multimesh().is_valid() ? st.node->get_multimesh()->get_instance_count() : 0;
		out["far"] = st.far_node && st.far_node->get_multimesh().is_valid() ? st.far_node->get_multimesh()->get_instance_count() : 0;
		PackedInt32Array sc;
		for (int c : st.species_counts) sc.push_back(c);
		out["species"] = sc;
		out["min_height"] = st.min_height;
		out["max_height"] = st.max_height;
		out["low_conifers"] = st.low_conifers;
		out["low_total"] = st.low_total;
		out["high_conifers"] = st.high_conifers;
		out["high_total"] = st.high_total;
		out["far_height_ratio"] = st.far_height_ratio;
		out["snapped"] = st.snapped;
		bool skipped_any = false;
		for (const auto &kv : tiles) {
			const auto colon = kv.first.find(':');
			if (colon == std::string::npos) continue;
			char want[64];
			std::snprintf(want, sizeof(want), "%d_%d", tile_e, tile_n);
			if (kv.first.substr(colon + 1) == want && kv.second.forest_fill_skipped) skipped_any = true;
		}
		out["forest_fill_skipped"] = skipped_any;   // any entry of the square (the forest file beside a surveyed tile)
		out["low_mean_height"] = st.low_total > 0 ? st.low_height_sum / st.low_total : 0.0f;
		out["alpine_mean_height"] = st.alpine_total > 0 ? st.alpine_height_sum / st.alpine_total : 0.0f;
		out["alpine_total"] = st.alpine_total;
		out["snap_pending"] = st.snap_pending;
		out["ground_min"] = st.ground_min;
		out["ground_max"] = st.ground_max;
	}
	return out;
}

PackedVector2Array S3DVegetationManager::tile_tree_positions(int tile_e, int tile_n) const
{
	PackedVector2Array out;
	const TileState *found = find_tile_state(tile_e, tile_n);
	if (found) {
		const auto &pos = found->positions;
		for (size_t i = 0; i + 1 < pos.size(); i += 2) out.push_back(Vector2(pos[i], pos[i + 1]));
	}
	return out;
}

int S3DVegetationManager::pending_tile_count() const
{
	int n = 0;
	for (const auto &kv : tiles) if (kv.second.loaded && (kv.second.snap_pending || kv.second.reloading)) n++;
	return n;
}

PackedInt32Array S3DVegetationManager::tile_instance_counts(int tile_e, int tile_n) const
{
	PackedInt32Array out;
	for (const auto &kv : tiles) {
		const std::string &id = kv.first;
		const auto colon = id.find(':');
		if (colon == std::string::npos) continue;
		char want[64];
		std::snprintf(want, sizeof(want), "%d_%d", tile_e, tile_n);
		if (id.substr(colon + 1) != want) continue;
		const TileState &st = kv.second;
		out.push_back(st.node && st.node->get_multimesh().is_valid() ? st.node->get_multimesh()->get_instance_count() : 0);
		out.push_back(st.far_node && st.far_node->get_multimesh().is_valid() ? st.far_node->get_multimesh()->get_instance_count() : 0);
		return out;
	}
	return out;
}

int S3DVegetationManager::get_active_tile_count() const { return (int)tiles.size(); }

} // namespace s3d
