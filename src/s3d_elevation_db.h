#ifndef S3D_ELEVATION_DB_H
#define S3D_ELEVATION_DB_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/templates/hash_map.hpp>

namespace godot
{

	// Provides O(1) elevation queries via bilinear interpolation over loaded
	// heightmap tiles. Used by the flight dynamics model to get ground elevation
	// at arbitrary world coordinates without any search overhead.
	class S3DElevationDB : public RefCounted
	{
		GDCLASS(S3DElevationDB, RefCounted);

	public:
		struct TileData {
			Ref<Image> heightmap;
			int width;
			int height;
		};

	private:
		HashMap<uint64_t, TileData> tiles;
		int tile_size = 1024;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;

		static uint64_t _tile_key(int tile_x, int tile_z);

	protected:
		static void _bind_methods();

	public:
		S3DElevationDB();
		~S3DElevationDB();

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void load_tile(int tile_x, int tile_z, Ref<Image> heightmap);
		void unload_tile(int tile_x, int tile_z);
		bool has_tile(int tile_x, int tile_z) const;

		double get_elevation(double x, double z) const;
		double get_elevation_safe(double x, double z, double default_val) const;
	};

}

#endif // S3D_ELEVATION_DB_H
