#ifndef S3D_ELEVATION_DB_H
#define S3D_ELEVATION_DB_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/image.hpp>

namespace godot
{

	// Provides O(1) elevation queries via bilinear interpolation over loaded
	// heightmap tiles. Used by the flight dynamics model to get ground elevation
	// at arbitrary world coordinates without any search overhead.
	class S3DElevationDB : public RefCounted
	{
		GDCLASS(S3DElevationDB, RefCounted);

	protected:
		static void _bind_methods();

	public:
		S3DElevationDB();
		~S3DElevationDB();

		// Query the terrain elevation at world coordinates (x, z).
		// Returns the bilinear-interpolated height, or 0.0 if no tile is loaded
		// for that location.
		double get_elevation(double x, double z) const;

		// Register a heightmap image for the tile at grid coordinates (tile_x, tile_z).
		void load_tile(int tile_x, int tile_z, Ref<Image> heightmap);
	};

}

#endif // S3D_ELEVATION_DB_H
