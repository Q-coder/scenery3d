#ifndef SCENERY3D_H
#define SCENERY3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include "s3d_elevation_db.h"

namespace godot
{

	class S3DTileManager;

	// Main entry point for the terrain system.
	// Owns the tile manager and elevation database, and coordinates
	// tile loading around the camera position.
	class Scenery3D : public Node3D
	{
		GDCLASS(Scenery3D, Node3D);

	private:
		int tile_size = 1024;
		int load_radius = 8;
		String data_path;

		S3DTileManager *tile_manager = nullptr;
		Ref<S3DElevationDB> elevation_db;

	protected:
		static void _bind_methods();

	public:
		Scenery3D();
		~Scenery3D();

		void _ready() override;
		void _process(double delta) override;

		// Query terrain elevation at world coordinates (x, z).
		double get_elevation(double x, double z) const;

		// Access the elevation database directly (e.g. for JSBSim integration).
		Ref<S3DElevationDB> get_elevation_db() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_load_radius(int p_radius);
		int get_load_radius() const;

		void set_data_path(const String &p_path);
		String get_data_path() const;
	};

}

#endif // SCENERY3D_H
