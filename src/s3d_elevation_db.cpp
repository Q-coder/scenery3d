#include "s3d_elevation_db.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <cmath>

using namespace godot;

S3DElevationDB::S3DElevationDB()
{
}

S3DElevationDB::~S3DElevationDB()
{
}

uint64_t S3DElevationDB::_tile_key(int tile_x, int tile_z)
{
	return ((uint64_t)(uint32_t)tile_x << 32) | (uint32_t)tile_z;
}

void S3DElevationDB::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DElevationDB::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DElevationDB::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DElevationDB::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DElevationDB::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DElevationDB::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DElevationDB::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("load_tile", "tile_x", "tile_z", "heightmap"), &S3DElevationDB::load_tile);
	ClassDB::bind_method(D_METHOD("unload_tile", "tile_x", "tile_z"), &S3DElevationDB::unload_tile);
	ClassDB::bind_method(D_METHOD("has_tile", "tile_x", "tile_z"), &S3DElevationDB::has_tile);

	ClassDB::bind_method(D_METHOD("get_elevation", "x", "z"), &S3DElevationDB::get_elevation);
	ClassDB::bind_method(D_METHOD("get_elevation_safe", "x", "z", "default_val"), &S3DElevationDB::get_elevation_safe);
}

void S3DElevationDB::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DElevationDB::get_tile_size() const
{
	return tile_size;
}

void S3DElevationDB::set_origin_east(double p_east)
{
	origin_east = p_east;
}

double S3DElevationDB::get_origin_east() const
{
	return origin_east;
}

void S3DElevationDB::set_origin_north(double p_north)
{
	origin_north = p_north;
}

double S3DElevationDB::get_origin_north() const
{
	return origin_north;
}

void S3DElevationDB::load_tile(int tile_x, int tile_z, Ref<Image> heightmap)
{
	ERR_FAIL_COND_MSG(heightmap.is_null(), "S3DElevationDB::load_tile: heightmap is null.");

	TileData td;
	td.heightmap = heightmap;
	td.width = heightmap->get_width();
	td.height = heightmap->get_height();

	uint64_t key = _tile_key(tile_x, tile_z);
	tiles[key] = td;

	UtilityFunctions::print("S3DElevationDB: loaded tile (", tile_x, ", ", tile_z,
		") size ", td.width, "x", td.height);
}

void S3DElevationDB::unload_tile(int tile_x, int tile_z)
{
	uint64_t key = _tile_key(tile_x, tile_z);
	tiles.erase(key);
}

bool S3DElevationDB::has_tile(int tile_x, int tile_z) const
{
	uint64_t key = _tile_key(tile_x, tile_z);
	return tiles.has(key);
}

double S3DElevationDB::get_elevation(double x, double z) const
{
	// Convert world coordinates to absolute LV95.
	// +X = West, so LV95_E = origin_east - x
	double lv95_e = origin_east - x;
	double lv95_n = z + origin_north;

	// Compute absolute tile indices (matching tile_key used in load_tile).
	int tile_x = (int)floor(lv95_e / tile_size);
	int tile_z = (int)floor(lv95_n / tile_size);

	uint64_t key = _tile_key(tile_x, tile_z);
	if (!tiles.has(key)) {
		return 0.0;
	}

	const TileData &td = tiles[key];
	int w = td.width;
	int h = td.height;

	// Local coordinates within the tile [0, tile_size).
	double local_x = lv95_e - (double)tile_x * tile_size;
	double local_z = lv95_n - (double)tile_z * tile_size;

	// Convert to pixel coordinates.
	double px = local_x / tile_size * (w - 1);
	double pz = local_z / tile_size * (h - 1);

	// Bilinear interpolation indices.
	int x0 = (int)floor(px);
	int z0 = (int)floor(pz);
	int x1 = (x0 + 1 < w) ? x0 + 1 : w - 1;
	int z1 = (z0 + 1 < h) ? z0 + 1 : h - 1;

	// Clamp to valid range.
	if (x0 < 0) x0 = 0;
	if (z0 < 0) z0 = 0;
	if (x0 >= w) x0 = w - 1;
	if (z0 >= h) z0 = h - 1;
	if (x1 >= w) x1 = w - 1;
	if (z1 >= h) z1 = h - 1;

	double fx = px - floor(px);
	double fz = pz - floor(pz);

	// Sample the four nearest pixels (red channel for FORMAT_RF heightmaps).
	double h00 = td.heightmap->get_pixel(x0, z0).r;
	double h10 = td.heightmap->get_pixel(x1, z0).r;
	double h01 = td.heightmap->get_pixel(x0, z1).r;
	double h11 = td.heightmap->get_pixel(x1, z1).r;

	// Bilinear interpolation.
	return h00 * (1.0 - fx) * (1.0 - fz)
		 + h10 * fx * (1.0 - fz)
		 + h01 * (1.0 - fx) * fz
		 + h11 * fx * fz;
}

double S3DElevationDB::get_elevation_safe(double x, double z, double default_val) const
{
	double lv95_e = origin_east - x;
	double lv95_n = z + origin_north;
	int tile_x = (int)floor(lv95_e / tile_size);
	int tile_z = (int)floor(lv95_n / tile_size);

	uint64_t key = _tile_key(tile_x, tile_z);
	if (!tiles.has(key)) {
		return default_val;
	}

	return get_elevation(x, z);
}
