#include "s3d_tile_manager.h"
#include "s3d_tile.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cmath>

using namespace godot;

S3DTileManager::S3DTileManager()
{
}

S3DTileManager::~S3DTileManager()
{
	// Tiles are children of this node, so Godot will free them when this node is freed.
	// Just clear our tracking map.
	active_tiles.clear();
	pending_loads.clear();
}

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
}

void S3DTileManager::_process(double delta)
{
	Viewport *vp = get_viewport();
	if (!vp) {
		return;
	}

	Camera3D *camera = vp->get_camera_3d();
	if (!camera) {
		return;
	}

	update_tiles(camera->get_global_position());
}

void S3DTileManager::update_tiles(Vector3 camera_pos)
{
	// Compute camera tile coordinates.
	int cam_tx = (int)std::floor(camera_pos.x / (double)tile_size);
	int cam_tz = (int)std::floor(camera_pos.z / (double)tile_size);

	// Build the set of required tiles and queue any missing ones for loading.
	// We use a temporary set to track what is needed for the unload pass.
	std::unordered_map<uint64_t, bool> required;

	for (int tx = cam_tx - load_radius; tx <= cam_tx + load_radius; tx++) {
		for (int tz = cam_tz - load_radius; tz <= cam_tz + load_radius; tz++) {
			uint64_t key = tile_key(tx, tz);
			required[key] = true;

			if (active_tiles.find(key) == active_tiles.end()) {
				// Check if it's already in pending_loads to avoid duplicates.
				bool already_pending = false;
				for (const auto &p : pending_loads) {
					if (p.first == tx && p.second == tz) {
						already_pending = true;
						break;
					}
				}
				if (!already_pending) {
					pending_loads.push_back(std::make_pair(tx, tz));
				}
			}
		}
	}

	// Unload tiles that are outside load_radius + unload_margin.
	int max_dist = load_radius + unload_margin;
	std::vector<uint64_t> to_remove;

	for (auto &pair : active_tiles) {
		S3DTile *tile = pair.second;
		int tx = tile->get_tile_x();
		int tz = tile->get_tile_z();

		if (std::abs(tx - cam_tx) > max_dist || std::abs(tz - cam_tz) > max_dist) {
			to_remove.push_back(pair.first);
		}
	}

	for (uint64_t key : to_remove) {
		auto it = active_tiles.find(key);
		if (it != active_tiles.end()) {
			it->second->queue_free();
			active_tiles.erase(it);
		}
	}

	// Process up to load_budget pending loads per frame.
	int loaded = 0;
	while (!pending_loads.empty() && loaded < load_budget) {
		auto [tx, tz] = pending_loads.back();
		pending_loads.pop_back();

		uint64_t key = tile_key(tx, tz);

		// Skip if tile was loaded in the meantime (e.g. duplicate in queue).
		if (active_tiles.find(key) != active_tiles.end()) {
			continue;
		}

		S3DTile *tile = memnew(S3DTile);
		tile->set_tile_x(tx);
		tile->set_tile_z(tz);
		tile->set_tile_size(tile_size);
		tile->set_position(Vector3((double)tx * tile_size, 0.0, (double)tz * tile_size));

		add_child(tile);

		// Try to load heightmap from disk.
		if (!data_path.is_empty()) {
			String path = data_path + "/tile_" + itos(tx) + "_" + itos(tz) + ".res";
			Ref<Resource> res = ResourceLoader::get_singleton()->load(path, "", ResourceLoader::CACHE_MODE_REUSE);
			if (res.is_valid()) {
				Ref<Image> heightmap = res;
				if (heightmap.is_valid()) {
					tile->set_heightmap(heightmap);
				}
			}
		}

		active_tiles[key] = tile;
		loaded++;
	}
}

int S3DTileManager::get_active_tile_count() const
{
	return (int)active_tiles.size();
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
