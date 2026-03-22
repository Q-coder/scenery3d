#include "s3d_tile.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

S3DTile::S3DTile()
{
}

S3DTile::~S3DTile()
{
}

void S3DTile::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("generate_mesh", "heightmap"), &S3DTile::generate_mesh);
	ClassDB::bind_method(D_METHOD("set_heightmap", "img"), &S3DTile::set_heightmap);

	ClassDB::bind_method(D_METHOD("set_tile_x", "x"), &S3DTile::set_tile_x);
	ClassDB::bind_method(D_METHOD("get_tile_x"), &S3DTile::get_tile_x);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_x"), "set_tile_x", "get_tile_x");

	ClassDB::bind_method(D_METHOD("set_tile_z", "z"), &S3DTile::set_tile_z);
	ClassDB::bind_method(D_METHOD("get_tile_z"), &S3DTile::get_tile_z);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_z"), "set_tile_z", "get_tile_z");

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DTile::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DTile::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");
}

void S3DTile::generate_mesh(Ref<Image> heightmap)
{
	// TODO: Generate an ArrayMesh from heightmap pixel data.
	// This should create a grid of vertices with Y values sampled
	// from the heightmap, plus normals computed from adjacent samples.
	// Target: run on a background thread, then apply mesh on main thread.
}

void S3DTile::set_heightmap(Ref<Image> img)
{
	// TODO: Store heightmap reference and trigger mesh regeneration.
}

void S3DTile::set_tile_x(int p_x)
{
	tile_x = p_x;
}

int S3DTile::get_tile_x() const
{
	return tile_x;
}

void S3DTile::set_tile_z(int p_z)
{
	tile_z = p_z;
}

int S3DTile::get_tile_z() const
{
	return tile_z;
}

void S3DTile::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DTile::get_tile_size() const
{
	return tile_size;
}
