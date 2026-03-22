#include "s3d_tile_manager.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

S3DTileManager::S3DTileManager()
{
}

S3DTileManager::~S3DTileManager()
{
}

void S3DTileManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("update_tiles", "camera_pos"), &S3DTileManager::update_tiles);

	ClassDB::bind_method(D_METHOD("set_load_budget", "budget"), &S3DTileManager::set_load_budget);
	ClassDB::bind_method(D_METHOD("get_load_budget"), &S3DTileManager::get_load_budget);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_budget"), "set_load_budget", "get_load_budget");

	ClassDB::bind_method(D_METHOD("set_unload_margin", "margin"), &S3DTileManager::set_unload_margin);
	ClassDB::bind_method(D_METHOD("get_unload_margin"), &S3DTileManager::get_unload_margin);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "unload_margin"), "set_unload_margin", "get_unload_margin");
}

void S3DTileManager::_process(double delta)
{
}

void S3DTileManager::update_tiles(Vector3 camera_pos)
{
	// TODO: Determine visible tile grid coordinates from camera_pos,
	// queue new tiles for background loading, and schedule distant
	// tiles for unloading (respecting unload_margin).
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
