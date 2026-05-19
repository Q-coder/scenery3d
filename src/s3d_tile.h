#ifndef S3D_TILE_H
#define S3D_TILE_H

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

namespace s3d
{

	// Represents a single terrain tile as a mesh generated from heightmap data.
	// The heightmap image should be FORMAT_RF (32-bit float per pixel) with
	// values representing elevation in meters. The mesh spans [0, tile_size]
	// in X and Z, with Y taken directly from pixel values.
	class S3DTile : public godot::MeshInstance3D
	{
		GDCLASS(S3DTile, godot::MeshInstance3D);

	private:
		int tile_x = 0;
		int tile_z = 0;
		int tile_size = 0;
		int lod_level = 4;
		bool mesh_ready = false;

		godot::Ref<godot::Image> heightmap;
		godot::Ref<godot::StandardMaterial3D> material;

		// Sample height at pixel coordinates, clamped to image bounds.
		godot::real_t sample_height(int px, int py, int img_w, int img_h) const;

	protected:
		static void _bind_methods();

	public:
		S3DTile();
		~S3DTile();

		// Build the terrain mesh from the stored heightmap.
		void generate_mesh();

		// Store the heightmap image (does NOT auto-generate mesh).
		void set_heightmap(godot::Ref<godot::Image> p_heightmap);
		godot::Ref<godot::Image> get_heightmap() const;

		// Set shared material (used by generate_mesh if set).
		void set_material(godot::Ref<godot::StandardMaterial3D> p_mat);
		godot::Ref<godot::StandardMaterial3D> get_material() const;

		// Returns true once the mesh has been generated.
		bool is_mesh_ready() const;

		void set_lod_level(int p_lod);
		int get_lod_level() const;

		void set_tile_x(int p_x);
		int get_tile_x() const;

		void set_tile_z(int p_z);
		int get_tile_z() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;
	};

}

#endif // S3D_TILE_H
