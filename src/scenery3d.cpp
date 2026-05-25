#include "scenery3d.h"
#include "s3d_tile_manager.h"
#include "s3d_building_manager.h"
#include "s3d_water_manager.h"
#include "s3d_road_manager.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

namespace s3d {

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

	ClassDB::bind_method(D_METHOD("set_tile_px", "px"), &Scenery3D::set_tile_px);
	ClassDB::bind_method(D_METHOD("get_tile_px"), &Scenery3D::get_tile_px);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_px"), "set_tile_px", "get_tile_px");

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

	ClassDB::bind_method(D_METHOD("set_data_paths", "paths"), &Scenery3D::set_data_paths);
	ClassDB::bind_method(D_METHOD("get_data_paths"), &Scenery3D::get_data_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "data_paths"), "set_data_paths", "get_data_paths");

	ClassDB::bind_method(D_METHOD("set_buildings_path", "path"), &Scenery3D::set_buildings_path);
	ClassDB::bind_method(D_METHOD("get_buildings_path"), &Scenery3D::get_buildings_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "buildings_path"), "set_buildings_path", "get_buildings_path");

	ClassDB::bind_method(D_METHOD("set_buildings_paths", "paths"), &Scenery3D::set_buildings_paths);
	ClassDB::bind_method(D_METHOD("get_buildings_paths"), &Scenery3D::get_buildings_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "buildings_paths"), "set_buildings_paths", "get_buildings_paths");

	ClassDB::bind_method(D_METHOD("set_water_paths", "paths"), &Scenery3D::set_water_paths);
	ClassDB::bind_method(D_METHOD("get_water_paths"), &Scenery3D::get_water_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "water_paths"), "set_water_paths", "get_water_paths");

	ClassDB::bind_method(D_METHOD("set_road_paths", "paths"), &Scenery3D::set_road_paths);
	ClassDB::bind_method(D_METHOD("get_road_paths"), &Scenery3D::get_road_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "road_paths"), "set_road_paths", "get_road_paths");

	ClassDB::bind_method(D_METHOD("set_orthophoto_path", "path"), &Scenery3D::set_orthophoto_path);
	ClassDB::bind_method(D_METHOD("get_orthophoto_path"), &Scenery3D::get_orthophoto_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "orthophoto_path"), "set_orthophoto_path", "get_orthophoto_path");

	ClassDB::bind_method(D_METHOD("set_orthophoto_paths", "paths"), &Scenery3D::set_orthophoto_paths);
	ClassDB::bind_method(D_METHOD("get_orthophoto_paths"), &Scenery3D::get_orthophoto_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "orthophoto_paths"), "set_orthophoto_paths", "get_orthophoto_paths");

	ClassDB::bind_method(D_METHOD("set_skip_white_pixels", "skip"), &Scenery3D::set_skip_white_pixels);
	ClassDB::bind_method(D_METHOD("get_skip_white_pixels"), &Scenery3D::get_skip_white_pixels);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "skip_white_pixels"), "set_skip_white_pixels", "get_skip_white_pixels");

	ClassDB::bind_method(D_METHOD("hide_building", "uuid"), &Scenery3D::hide_building);
	ClassDB::bind_method(D_METHOD("show_building", "uuid"), &Scenery3D::show_building);
	ClassDB::bind_method(D_METHOD("is_building_hidden", "uuid"), &Scenery3D::is_building_hidden);
}

void Scenery3D::_ready()
{
	// Resolve data_paths: if empty, fall back to data_path (single),
	// then to project setting "scenery3d/data_path".
	if (data_paths.is_empty() && !data_path.is_empty()) {
		data_paths.push_back(data_path);
	}
	if (data_paths.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/data_path")) {
			String p = ps->get_setting("scenery3d/data_path");
			if (!p.is_empty()) {
				data_path = p;
				data_paths.push_back(p);
			}
		}
	}
	if (data_paths.is_empty()) {
		UtilityFunctions::push_warning("Scenery3D: data_path(s) not set. Set data_path or data_paths in the inspector or via project setting 'scenery3d/data_path'.");
	}

	// Create and configure the tile manager.
	tile_manager = memnew(S3DTileManager);
	add_child(tile_manager);
	tile_manager->set_tile_size(tile_size);
	tile_manager->set_tile_px(tile_px);
	tile_manager->set_load_radius(load_radius);
	tile_manager->set_far_radius(far_radius);
	tile_manager->set_data_paths(data_paths);
	tile_manager->set_origin_east(origin_east);
	tile_manager->set_origin_north(origin_north);
	// Resolve orthophoto_paths: array wins, then single path, then project setting.
	if (orthophoto_paths.is_empty() && !orthophoto_path.is_empty()) {
		orthophoto_paths.push_back(orthophoto_path);
	}
	if (orthophoto_paths.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/orthophoto_paths")) {
			Variant ov = ps->get_setting("scenery3d/orthophoto_paths");
			if (ov.get_type() == Variant::PACKED_STRING_ARRAY) orthophoto_paths = ov;
		}
	}
	if (!orthophoto_paths.is_empty()) {
		tile_manager->set_orthophoto_paths(orthophoto_paths);
	}
	tile_manager->set_skip_white_pixels(skip_white_pixels);

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
	// Resolve buildings_paths: if empty, fall back to buildings_path (single),
	// then to ProjectSettings.
	if (buildings_paths.is_empty() && !buildings_path.is_empty()) {
		buildings_paths.push_back(buildings_path);
	}
	if (buildings_paths.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/buildings_path")) {
			String p = ps->get_setting("scenery3d/buildings_path");
			if (!p.is_empty()) {
				buildings_paths.push_back(p);
			}
		}
	}
	if (!buildings_paths.is_empty()) {
		building_manager = memnew(S3DBuildingManager);
		add_child(building_manager);
		building_manager->set_buildings_paths(buildings_paths);
		building_manager->set_origin_east(origin_east);
		building_manager->set_origin_north(origin_north);
	}

	// Create the water manager.
	if (water_paths.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/water_paths")) {
			Variant wv = ps->get_setting("scenery3d/water_paths");
			if (wv.get_type() == Variant::PACKED_STRING_ARRAY) {
				water_paths = wv;
			}
		}
	}
	if (!water_paths.is_empty()) {
		water_manager = memnew(S3DWaterManager);
		add_child(water_manager);
		water_manager->set_origin_east(origin_east);
		water_manager->set_origin_north(origin_north);
		water_manager->set_elevation_db(elevation_db);
		water_manager->set_water_paths(water_paths);
	}

	// Create the road manager.
	if (road_paths.is_empty()) {
		ProjectSettings *ps = ProjectSettings::get_singleton();
		if (ps && ps->has_setting("scenery3d/road_paths")) {
			Variant rv = ps->get_setting("scenery3d/road_paths");
			if (rv.get_type() == Variant::PACKED_STRING_ARRAY) {
				road_paths = rv;
			}
		}
	}
	if (!road_paths.is_empty()) {
		road_manager = memnew(S3DRoadManager);
		add_child(road_manager);
		road_manager->set_origin_east(origin_east);
		road_manager->set_origin_north(origin_north);
		road_manager->set_elevation_db(elevation_db);
		road_manager->set_road_paths(road_paths);
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

void Scenery3D::set_tile_px(int p_px)
{
	tile_px = p_px;
	if (tile_manager) {
		tile_manager->set_tile_px(p_px);
	}
}

int Scenery3D::get_tile_px() const
{
	return tile_px;
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

void Scenery3D::set_data_paths(const PackedStringArray &p_paths)
{
	data_paths = p_paths;
	if (tile_manager) {
		tile_manager->set_data_paths(p_paths);
	}
}

PackedStringArray Scenery3D::get_data_paths() const
{
	return data_paths;
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

void Scenery3D::set_buildings_paths(const PackedStringArray &p_paths)
{
	buildings_paths = p_paths;
	if (building_manager) {
		building_manager->set_buildings_paths(p_paths);
	}
}

PackedStringArray Scenery3D::get_buildings_paths() const
{
	return buildings_paths;
}

void Scenery3D::set_water_paths(const PackedStringArray &p_paths)
{
	water_paths = p_paths;
	if (water_manager) {
		water_manager->set_water_paths(p_paths);
	}
}

PackedStringArray Scenery3D::get_water_paths() const
{
	return water_paths;
}

void Scenery3D::set_road_paths(const PackedStringArray &p_paths)
{
	road_paths = p_paths;
	if (road_manager) {
		road_manager->set_road_paths(p_paths);
	}
}

PackedStringArray Scenery3D::get_road_paths() const
{
	return road_paths;
}

void Scenery3D::set_orthophoto_path(const String &p_path)
{
	orthophoto_path = p_path;
	orthophoto_paths.clear();
	if (!p_path.is_empty()) orthophoto_paths.push_back(p_path);
	if (tile_manager) {
		tile_manager->set_orthophoto_paths(orthophoto_paths);
	}
}

String Scenery3D::get_orthophoto_path() const
{
	if (!orthophoto_paths.is_empty()) return orthophoto_paths[0];
	return orthophoto_path;
}

void Scenery3D::set_orthophoto_paths(const PackedStringArray &p_paths)
{
	orthophoto_paths = p_paths;
	orthophoto_path = p_paths.is_empty() ? String() : p_paths[0];
	if (tile_manager) {
		tile_manager->set_orthophoto_paths(p_paths);
	}
}

PackedStringArray Scenery3D::get_orthophoto_paths() const
{
	return orthophoto_paths;
}

void Scenery3D::set_skip_white_pixels(bool p_skip)
{
	skip_white_pixels = p_skip;
	if (tile_manager) {
		tile_manager->set_skip_white_pixels(p_skip);
	}
}

bool Scenery3D::get_skip_white_pixels() const
{
	return skip_white_pixels;
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
	if (building_manager) {
		building_manager->set_origin_east(p_east);
	}
	if (water_manager) {
		water_manager->set_origin_east(p_east);
	}
	if (road_manager) {
		road_manager->set_origin_east(p_east);
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
	if (building_manager) {
		building_manager->set_origin_north(p_north);
	}
	if (water_manager) {
		water_manager->set_origin_north(p_north);
	}
	if (road_manager) {
		road_manager->set_origin_north(p_north);
	}
}

double Scenery3D::get_origin_north() const
{
	return origin_north;
}

} // namespace s3d
