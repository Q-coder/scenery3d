#include "scenery3d.h"
#include "s3d_tile_manager.h"
#include "s3d_building_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

Scenery3D::Scenery3D()
{
}

Scenery3D::~Scenery3D()
{
	// tile_manager is a child node, Godot frees it automatically.
	// elevation_db is Ref-counted, released automatically.
}

void Scenery3D::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_elevation", "x", "z"), &Scenery3D::get_elevation);
	ClassDB::bind_method(D_METHOD("get_elevation_db"), &Scenery3D::get_elevation_db);
	ClassDB::bind_method(D_METHOD("get_coords"), &Scenery3D::get_coords);

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &Scenery3D::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &Scenery3D::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_load_radius", "radius"), &Scenery3D::set_load_radius);
	ClassDB::bind_method(D_METHOD("get_load_radius"), &Scenery3D::get_load_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius"), "set_load_radius", "get_load_radius");

	ClassDB::bind_method(D_METHOD("set_far_radius", "radius"), &Scenery3D::set_far_radius);
	ClassDB::bind_method(D_METHOD("get_far_radius"), &Scenery3D::get_far_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "far_radius"), "set_far_radius", "get_far_radius");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &Scenery3D::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &Scenery3D::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &Scenery3D::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &Scenery3D::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_data_path", "path"), &Scenery3D::set_data_path);
	ClassDB::bind_method(D_METHOD("get_data_path"), &Scenery3D::get_data_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "data_path"), "set_data_path", "get_data_path");

	ClassDB::bind_method(D_METHOD("set_buildings_path", "path"), &Scenery3D::set_buildings_path);
	ClassDB::bind_method(D_METHOD("get_buildings_path"), &Scenery3D::get_buildings_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "buildings_path"), "set_buildings_path", "get_buildings_path");

	ClassDB::bind_method(D_METHOD("hide_building", "uuid"), &Scenery3D::hide_building);
	ClassDB::bind_method(D_METHOD("show_building", "uuid"), &Scenery3D::show_building);
	ClassDB::bind_method(D_METHOD("is_building_hidden", "uuid"), &Scenery3D::is_building_hidden);
}

void Scenery3D::_ready()
{
	// Resolve data_path: use project setting "scenery3d/data_path" as fallback.
	if (data_path.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/data_path")) {
			data_path = ps->get_setting("scenery3d/data_path");
		}
	}
	if (data_path.is_empty()) {
		UtilityFunctions::push_warning("Scenery3D: data_path is not set. Set it in the inspector or via project setting 'scenery3d/data_path'.");
	}

	// Create and configure the tile manager.
	tile_manager = memnew(S3DTileManager);
	add_child(tile_manager);
	tile_manager->set_tile_size(tile_size);
	tile_manager->set_load_radius(load_radius);
	tile_manager->set_far_radius(far_radius);
	tile_manager->set_data_path(data_path);
	tile_manager->set_origin_east(origin_east);
	tile_manager->set_origin_north(origin_north);

	// Create the elevation database and share it with the tile manager.
	elevation_db.instantiate();
	elevation_db->set_tile_size(tile_size);
	elevation_db->set_origin_east(origin_east);
	elevation_db->set_origin_north(origin_north);
	tile_manager->set_elevation_db(elevation_db);

	// Create the coordinate mapper.
	coords.instantiate();
	coords->set_origin_east(origin_east);
	coords->set_origin_north(origin_north);

	// Create the building manager.
	if (!buildings_path.is_empty()) {
		building_manager = memnew(S3DBuildingManager);
		add_child(building_manager);
		building_manager->set_buildings_path(buildings_path);
		building_manager->set_origin_east(origin_east);
		building_manager->set_origin_north(origin_north);
	} else {
		// Try project setting fallback.
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/buildings_path")) {
			buildings_path = ps->get_setting("scenery3d/buildings_path");
			if (!buildings_path.is_empty()) {
				building_manager = memnew(S3DBuildingManager);
				add_child(building_manager);
				building_manager->set_buildings_path(buildings_path);
				building_manager->set_origin_east(origin_east);
				building_manager->set_origin_north(origin_north);
			}
		}
	}
}

void Scenery3D::_process(double delta)
{
	// The tile manager handles its own _process for tile loading/unloading.
}

double Scenery3D::get_elevation(double x, double z) const
{
	if (elevation_db.is_valid()) {
		return elevation_db->get_elevation(x, z);
	}
	return 0.0;
}

Ref<S3DElevationDB> Scenery3D::get_elevation_db() const
{
	return elevation_db;
}

Ref<S3DCoords> Scenery3D::get_coords() const
{
	return coords;
}

void Scenery3D::set_tile_size(int p_size)
{
	tile_size = p_size;
	if (tile_manager) {
		tile_manager->set_tile_size(p_size);
	}
}

int Scenery3D::get_tile_size() const
{
	return tile_size;
}

void Scenery3D::set_load_radius(int p_radius)
{
	load_radius = p_radius;
	if (tile_manager) {
		tile_manager->set_load_radius(p_radius);
	}
}

int Scenery3D::get_load_radius() const
{
	return load_radius;
}

void Scenery3D::set_far_radius(int p_radius)
{
	far_radius = p_radius;
	if (tile_manager) {
		tile_manager->set_far_radius(p_radius);
	}
}

int Scenery3D::get_far_radius() const
{
	return far_radius;
}

void Scenery3D::set_data_path(const String &p_path)
{
	data_path = p_path;
	if (tile_manager) {
		tile_manager->set_data_path(p_path);
	}
}

String Scenery3D::get_data_path() const
{
	return data_path;
}

void Scenery3D::set_buildings_path(const String &p_path)
{
	buildings_path = p_path;
	if (building_manager) {
		building_manager->set_buildings_path(p_path);
	}
}

String Scenery3D::get_buildings_path() const
{
	return buildings_path;
}

void Scenery3D::hide_building(const String &uuid)
{
	if (building_manager) {
		building_manager->hide_building(uuid);
	}
}

void Scenery3D::show_building(const String &uuid)
{
	if (building_manager) {
		building_manager->show_building(uuid);
	}
}

bool Scenery3D::is_building_hidden(const String &uuid) const
{
	if (building_manager) {
		return building_manager->is_building_hidden(uuid);
	}
	return false;
}

void Scenery3D::set_origin_east(double p_east)
{
	origin_east = p_east;
	if (tile_manager) {
		tile_manager->set_origin_east(p_east);
	}
	if (elevation_db.is_valid()) {
		elevation_db->set_origin_east(p_east);
	}
	if (coords.is_valid()) {
		coords->set_origin_east(p_east);
	}
}

double Scenery3D::get_origin_east() const
{
	return origin_east;
}

void Scenery3D::set_origin_north(double p_north)
{
	origin_north = p_north;
	if (tile_manager) {
		tile_manager->set_origin_north(p_north);
	}
	if (elevation_db.is_valid()) {
		elevation_db->set_origin_north(p_north);
	}
	if (coords.is_valid()) {
		coords->set_origin_north(p_north);
	}
}

double Scenery3D::get_origin_north() const
{
	return origin_north;
}
