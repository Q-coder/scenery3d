#include "s3d_tile.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/mesh.hpp>

using namespace godot;

S3DTile::S3DTile()
{
}

S3DTile::~S3DTile()
{
}

void S3DTile::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("generate_mesh"), &S3DTile::generate_mesh);

	ClassDB::bind_method(D_METHOD("set_heightmap", "heightmap"), &S3DTile::set_heightmap);
	ClassDB::bind_method(D_METHOD("get_heightmap"), &S3DTile::get_heightmap);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "heightmap", PROPERTY_HINT_RESOURCE_TYPE, "Image"), "set_heightmap", "get_heightmap");

	ClassDB::bind_method(D_METHOD("is_mesh_ready"), &S3DTile::is_mesh_ready);

	ClassDB::bind_method(D_METHOD("set_lod_level", "lod"), &S3DTile::set_lod_level);
	ClassDB::bind_method(D_METHOD("get_lod_level"), &S3DTile::get_lod_level);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "lod_level"), "set_lod_level", "get_lod_level");

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

real_t S3DTile::sample_height(int px, int py, int img_w, int img_h) const
{
	px = px < 0 ? 0 : (px >= img_w ? img_w - 1 : px);
	py = py < 0 ? 0 : (py >= img_h ? img_h - 1 : py);
	return static_cast<real_t>(heightmap->get_pixel(px, py).r);
}

void S3DTile::generate_mesh()
{
	if (heightmap.is_null() || tile_size <= 0) {
		return;
	}

	int img_w = heightmap->get_width();
	int img_h = heightmap->get_height();

	if (img_w < 2 || img_h < 2) {
		return;
	}

	// Compute stride from LOD level. Higher LOD = coarser mesh.
	int stride = 1 << lod_level;

	// Number of vertices along each axis after applying stride.
	int verts_x = (img_w - 1) / stride + 1;
	int verts_z = (img_h - 1) / stride + 1;

	if (verts_x < 2 || verts_z < 2) {
		return;
	}

	int vertex_count = verts_x * verts_z;
	int quad_count = (verts_x - 1) * (verts_z - 1);
	int index_count = quad_count * 6;

	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedVector2Array uvs;
	PackedInt32Array indices;

	vertices.resize(vertex_count);
	normals.resize(vertex_count);
	uvs.resize(vertex_count);
	indices.resize(index_count);

	real_t ts = static_cast<real_t>(tile_size);

	// Spacing between vertices in world units.
	real_t spacing_x = ts / static_cast<real_t>(verts_x - 1);
	real_t spacing_z = ts / static_cast<real_t>(verts_z - 1);

	// Spacing in pixel units (used for normal computation via central differences).
	real_t pixel_spacing_x = ts / static_cast<real_t>(img_w - 1);
	real_t pixel_spacing_z = ts / static_cast<real_t>(img_h - 1);

	// Generate vertices, normals, and UVs.
	for (int gz = 0; gz < verts_z; gz++) {
		int py = gz * stride;
		if (py >= img_h) {
			py = img_h - 1;
		}

		for (int gx = 0; gx < verts_x; gx++) {
			int px = gx * stride;
			if (px >= img_w) {
				px = img_w - 1;
			}

			int idx = gz * verts_x + gx;

			real_t world_x = static_cast<real_t>(gx) * spacing_x;
			real_t world_z = static_cast<real_t>(gz) * spacing_z;
			real_t height = sample_height(px, py, img_w, img_h);

			vertices[idx] = Vector3(world_x, height, world_z);

			// UV: [0,1] range across the tile.
			real_t u = static_cast<real_t>(gx) / static_cast<real_t>(verts_x - 1);
			real_t v = static_cast<real_t>(gz) / static_cast<real_t>(verts_z - 1);
			uvs[idx] = Vector2(u, v);

			// Compute smooth normal using central differences on the heightmap.
			// Sample one stride step in each direction for consistent spacing.
			real_t h_left = sample_height(px - stride, py, img_w, img_h);
			real_t h_right = sample_height(px + stride, py, img_w, img_h);
			real_t h_down = sample_height(px, py - stride, img_w, img_h);
			real_t h_up = sample_height(px, py + stride, img_w, img_h);

			// The distance in world units between the two samples.
			real_t dx = static_cast<real_t>(stride) * pixel_spacing_x * 2.0;
			real_t dz = static_cast<real_t>(stride) * pixel_spacing_z * 2.0;

			// At boundaries the clamping reduces the actual sample distance,
			// but the normal approximation is still reasonable.
			Vector3 normal = Vector3(
				(h_left - h_right) / dx,
				1.0,
				(h_down - h_up) / dz
			);
			normals[idx] = normal.normalized();
		}
	}

	// Generate triangle indices. Two triangles per quad, wound counter-clockwise
	// so normals face upward (matching Godot's front-face convention).
	int ii = 0;
	for (int gz = 0; gz < verts_z - 1; gz++) {
		for (int gx = 0; gx < verts_x - 1; gx++) {
			int top_left = gz * verts_x + gx;
			int top_right = top_left + 1;
			int bottom_left = (gz + 1) * verts_x + gx;
			int bottom_right = bottom_left + 1;

			// First triangle (top-left, bottom-left, top-right).
			indices[ii++] = top_left;
			indices[ii++] = bottom_left;
			indices[ii++] = top_right;

			// Second triangle (top-right, bottom-left, bottom-right).
			indices[ii++] = top_right;
			indices[ii++] = bottom_left;
			indices[ii++] = bottom_right;
		}
	}

	// Build the ArrayMesh.
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = vertices;
	arrays[Mesh::ARRAY_NORMAL] = normals;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_INDEX] = indices;

	Ref<ArrayMesh> array_mesh;
	array_mesh.instantiate();
	array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

	set_mesh(array_mesh);
	mesh_ready = true;
}

void S3DTile::set_heightmap(Ref<Image> p_heightmap)
{
	heightmap = p_heightmap;
	mesh_ready = false;
	generate_mesh();
}

Ref<Image> S3DTile::get_heightmap() const
{
	return heightmap;
}

bool S3DTile::is_mesh_ready() const
{
	return mesh_ready;
}

void S3DTile::set_lod_level(int p_lod)
{
	lod_level = p_lod < 0 ? 0 : p_lod;
}

int S3DTile::get_lod_level() const
{
	return lod_level;
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
