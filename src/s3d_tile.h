#ifndef S3D_TILE_H
#define S3D_TILE_H

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/image.hpp>

namespace godot
{

	// Represents a single terrain tile as a mesh generated from heightmap data.
	// Mesh generation will be performed asynchronously on a background thread
	// to avoid blocking the main thread during tile loading.
	class S3DTile : public MeshInstance3D
	{
		GDCLASS(S3DTile, MeshInstance3D);

	private:
		int tile_x = 0;
		int tile_z = 0;
		int tile_size = 0;

	protected:
		static void _bind_methods();

	public:
		S3DTile();
		~S3DTile();

		// Build the terrain mesh from a heightmap image.
		void generate_mesh(Ref<Image> heightmap);

		// Set the heightmap data for this tile.
		void set_heightmap(Ref<Image> img);

		void set_tile_x(int p_x);
		int get_tile_x() const;

		void set_tile_z(int p_z);
		int get_tile_z() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;
	};

}

#endif // S3D_TILE_H
