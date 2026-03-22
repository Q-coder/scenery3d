#include "scenery3d.h"
#include "s3d_tile_manager.h"

#include <godot_cpp/core/class_db.hpp>
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

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &Scenery3D::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &Scenery3D::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_load_radius", "radius"), &Scenery3D::set_load_radius);
	ClassDB::bind_method(D_METHOD("get_load_radius"), &Scenery3D::get_load_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius"), "set_load_radius", "get_load_radius");

	ClassDB::bind_method(D_METHOD("set_data_path", "path"), &Scenery3D::set_data_path);
	ClassDB::bind_method(D_METHOD("get_data_path"), &Scenery3D::get_data_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "data_path"), "set_data_path", "get_data_path");
}

void Scenery3D::_ready()
{
	// Create and configure the tile manager.
	tile_manager = memnew(S3DTileManager);
	add_child(tile_manager);
	tile_manager->set_tile_size(tile_size);
	tile_manager->set_load_radius(load_radius);
	tile_manager->set_data_path(data_path);

	// Create the elevation database.
	elevation_db.instantiate();
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
