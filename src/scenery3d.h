#ifndef SCENERY3D_H
#define SCENERY3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include "s3d_elevation_db.h"
#include "s3d_coords.h"

namespace s3d
{

	class S3DTileManager;
	class S3DBuildingManager;
	class S3DWaterManager;
	class S3DRoadManager;

	// Main entry point for the terrain system.
	// Owns the tile manager and elevation database, and coordinates
	// tile loading around the camera position.
	class Scenery3D : public godot::Node3D
	{
		GDCLASS(Scenery3D, godot::Node3D);

	private:
		int tile_size = 1024;
		int load_radius = 8;
		int far_radius = 200;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		godot::String data_path;
		godot::PackedStringArray data_paths;
		godot::String buildings_path;
		godot::PackedStringArray buildings_paths;
		godot::PackedStringArray water_paths;
		godot::PackedStringArray road_paths;
		godot::String orthophoto_path;
		godot::PackedStringArray orthophoto_paths;
		bool skip_white_pixels = true;

		S3DTileManager *tile_manager = nullptr;
		S3DBuildingManager *building_manager = nullptr;
		S3DWaterManager *water_manager = nullptr;
		S3DRoadManager *road_manager = nullptr;
		godot::Ref<S3DElevationDB> elevation_db;
		godot::Ref<S3DCoords> coords;

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
		godot::Ref<S3DElevationDB> get_elevation_db() const;

		godot::Ref<S3DCoords> get_coords() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_load_radius(int p_radius);
		int get_load_radius() const;

		void set_far_radius(int p_radius);
		int get_far_radius() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_data_path(const godot::String &p_path);
		godot::String get_data_path() const;

		void set_data_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_data_paths() const;

		void set_buildings_path(const godot::String &p_path);
		godot::String get_buildings_path() const;

		void set_buildings_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_buildings_paths() const;

		void set_water_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_water_paths() const;

		void set_road_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_road_paths() const;

		void set_orthophoto_path(const godot::String &p_path);
		godot::String get_orthophoto_path() const;

		void set_orthophoto_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_orthophoto_paths() const;

		void set_skip_white_pixels(bool p_skip);
		bool get_skip_white_pixels() const;

		// Building visibility control.
		void hide_building(const godot::String &uuid);
		void show_building(const godot::String &uuid);
		bool is_building_hidden(const godot::String &uuid) const;
	};

}

#endif // SCENERY3D_H
