#include "s3d_elevation_db.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

S3DElevationDB::S3DElevationDB()
{
}

S3DElevationDB::~S3DElevationDB()
{
}

void S3DElevationDB::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("get_elevation", "x", "z"), &S3DElevationDB::get_elevation);
	ClassDB::bind_method(D_METHOD("load_tile", "tile_x", "tile_z", "heightmap"), &S3DElevationDB::load_tile);
}

double S3DElevationDB::get_elevation(double x, double z) const
{
	// TODO: Convert world (x, z) to tile grid coordinates, look up the
	// corresponding heightmap, then bilinear-interpolate the four nearest
	// pixels for an O(1) elevation query.
	return 0.0;
}

void S3DElevationDB::load_tile(int tile_x, int tile_z, Ref<Image> heightmap)
{
	// TODO: Store the heightmap in an internal map keyed by (tile_x, tile_z)
	// for fast lookup by get_elevation().
}
