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
			if (it->second.far_node) {
				it->second.far_node->queue_free();
			}
			it = tiles.erase(it);
		} else {
			++it;
		}
	}
}

// ── Card mode assets ────────────────────────────────────────────────────────
// Three vertical quads at 60 deg and one canopy quad: unit height (base at
// y = 0), unit crown width. UV = card space (v = 0 at the top of the card),
// COLOR.r = 1 on the canopy quad. The shader picks the species column.
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
		const double a = k * (M_PI / 3.0);
		const Vector3 d(0.5 * std::cos(a), 0.0, 0.5 * std::sin(a));
		quad(-d, d, d + Vector3(0, 1, 0), -d + Vector3(0, 1, 0), 0.0f);
	}
	const float cy = 0.72f;
	quad(Vector3(-0.5f, cy, -0.5f), Vector3(0.5f, cy, -0.5f), Vector3(0.5f, cy, 0.5f), Vector3(-0.5f, cy, 0.5f), 1.0f);
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
	} else {
		UtilityFunctions::push_warning("S3DVegetationManager: shaders/tree.gdshader not found; cards draw untextured");
	}
}

// Conifers take over with altitude (Swiss treeline mix: beech country below
// 800 m, spruce / larch above 1 400 m); the hash spreads species evenly inside
// each group.
int S3DVegetationManager::pick_species(uint32_t hash, float ground_z) const
{
	if (species.size() <= 1) return 0;
	const double p_conifer = std::clamp(0.25 + (ground_z - 700.0) / 900.0, 0.2, 0.97);
	const bool conifer = ((hash & 0xFFFF) / 65535.0) < p_conifer;
	std::vector<int> pool;
	for (size_t i = 0; i < species.size(); i++) if (species[i].conifer == conifer) pool.push_back((int)i);
	if (pool.empty()) for (size_t i = 0; i < species.size(); i++) pool.push_back((int)i);
	return pool[(hash >> 16) % pool.size()];
}

// One MultiMeshInstance3D for a subset of a tile's records. Card mode: the
// card mesh, scaled to the tree's height and crown, with the species /
// variation in the custom data; mesh mode: the user mesh as before.
MultiMeshInstance3D *S3DVegetationManager::make_instances(const LoadResult &result, const std::vector<size_t> &indices, double scale_mul, const char *suffix, TileState *stats)
{
	if (stats) {
		stats->species_counts.assign(std::max<size_t>(1, species.size()), 0);
		stats->min_height = 1e9f;
		stats->max_height = 0.0f;
		stats->low_conifers = stats->high_conifers = stats->low_total = stats->high_total = 0;
		stats->snapped = 0;
		stats->ground_min = 1e9f;
		stats->ground_max = -1e9f;
		stats->snap_pending = false;
	}
	const bool snap = card_mode && snap_to_terrain && elevation_db.is_valid();
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
			const double h = elevation_db->get_elevation_grid(result.tile_east + (double)t.dx, result.tile_north + (double)t.dz);
			if (std::isnan(h)) {
				if (stats) stats->snap_pending = true;
			} else {
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
			const int sp = pick_species(h, t.ground_z);
			const Species &S = species[sp];
			scale_y = t.height_m * j * (float)scale_mul;
			// the card is one crown width wide: from the record's crown, else the species ratio
			const float crown_w = t.crown_radius_m > 0.1f ? 2.0f * t.crown_radius_m : (float)(S.crown_ratio * t.height_m);
			scale_xz = std::max(crown_w, 0.35f * t.height_m) * j * (float)scale_mul;
			b = b.rotated(Vector3(0, 1, 0), t.yaw_rad);
			b = b.scaled(Vector3(scale_xz, scale_y, scale_xz));
			mm->set_instance_custom_data((int)k, Color((float)sp, (float)((h >> 8) & 0xFF) / 255.0f, (float)((h >> 20) & 0xFF) / 255.0f, 0.0f));
			if (stats) {
				stats->species_counts[sp]++;
				stats->min_height = std::min(stats->min_height, scale_y);
				stats->max_height = std::max(stats->max_height, scale_y);
				if (t.ground_z < 700.0f) { stats->low_total++; if (S.conifer) stats->low_conifers++; }
				if (t.ground_z > 1300.0f) { stats->high_total++; if (S.conifer) stats->high_conifers++; }
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
			st.node = make_instances(result, keep_indices, 1.0, "", nullptr);
			continue;
		}
		// Card mode: the near set (every kept tree) to near_range_m, the far
		// set (every n-th tree, enlarged so the canopy cover stays) beyond.
		st.node = make_instances(result, keep_indices, 1.0, "_near", &st);
		st.node->set_visibility_range_end((float)near_range_m);
		st.node->set_visibility_range_fade_mode(GeometryInstance3D::VISIBILITY_RANGE_FADE_DISABLED);
		const int step = std::max(1, (int)std::lround(1.0 / std::clamp(far_fraction, 0.02, 1.0)));
		std::vector<size_t> far_indices;
		far_indices.reserve(keep_indices.size() / step + 1);
		for (size_t k = 0; k < keep_indices.size(); k += step) far_indices.push_back(keep_indices[k]);
		st.far_node = make_instances(result, far_indices, far_scale, "_far", nullptr);
		st.far_height_ratio = (float)far_scale;
		st.far_node->set_visibility_range_begin((float)near_range_m);
		st.far_node->set_visibility_range_end((float)far_range_m + 1500.0f);
		st.far_node->set_visibility_range_fade_mode(GeometryInstance3D::VISIBILITY_RANGE_FADE_DISABLED);
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
		if (!elevation_db->has_tile((int)std::floor((te + 512.0) / ts), (int)std::floor((tn + 512.0) / ts))) { ++it; continue; }
		if (st.node) st.node->queue_free();
		if (st.far_node) st.far_node->queue_free();
		it = tiles.erase(it);
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
	ClassDB::bind_method(D_METHOD("get_snap_to_terrain"), &S3DVegetationManager::get_snap_to_terrain);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "snap_to_terrain"), "set_snap_to_terrain", "get_snap_to_terrain");
	ClassDB::bind_method(D_METHOD("get_card_mesh"), &S3DVegetationManager::get_card_mesh);
	ClassDB::bind_method(D_METHOD("tile_instance_counts", "tile_e", "tile_n"), &S3DVegetationManager::tile_instance_counts);
	ClassDB::bind_method(D_METHOD("tile_stats", "tile_e", "tile_n"), &S3DVegetationManager::tile_stats);
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
void S3DVegetationManager::set_near_range_m(double v) { near_range_m = std::max(200.0, v); clear_loaded_tiles(); }
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
Ref<Mesh> S3DVegetationManager::get_card_mesh() { prepare_card_assets(); return card_mesh; }
Dictionary S3DVegetationManager::tile_stats(int tile_e, int tile_n) const
{
	Dictionary out;
	char want[64];
	std::snprintf(want, sizeof(want), "%d_%d", tile_e, tile_n);
	for (const auto &kv : tiles) {
		const auto colon = kv.first.find(':');
		if (colon == std::string::npos || kv.first.substr(colon + 1) != want) continue;
		const TileState &st = kv.second;
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
		out["snap_pending"] = st.snap_pending;
		out["ground_min"] = st.ground_min;
		out["ground_max"] = st.ground_max;
		return out;
	}
	return out;
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
