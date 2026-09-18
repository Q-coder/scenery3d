#ifndef S3D_COORDS_H
#define S3D_COORDS_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include "s3d_frame.h"

namespace s3d
{

	// Converts between WGS84 (latitude/longitude), Swiss LV95/CH1903+ (EPSG:2056),
	// and Godot world coordinates. Uses the rigorous projection in s3d_proj.h
	// (matches pyproj EPSG:2056 to the millimetre across Europe).
	//
	// Godot mapping (frame_mode 0, flat):
	//   X = origin_east - LV95 East    (positive = WEST)
	//   Y = elevation
	//   Z = LV95 North - origin_north  (positive = north)
	// frame_mode 1 (ECEF): world = WGS84 ECEF metres (see S3DFrame).
	class S3DCoords : public godot::RefCounted
	{
		GDCLASS(S3DCoords, godot::RefCounted);

	private:
		// LV95 origin that maps to Godot world (0, 0, 0).
		double origin_east = 2600000.0;  // Default: approximate center of Switzerland
		double origin_north = 1200000.0;
		S3DFrame frame;

	protected:
		static void _bind_methods();

	public:
		S3DCoords();
		~S3DCoords();

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		// Set origin from WGS84 lat/lon (degrees).
		void set_origin_from_wgs84(double latitude, double longitude);

		// 0 = flat legacy frame, 1 = ECEF (see S3DFrame).
		void set_frame_mode(int p_mode);
		int get_frame_mode() const { return frame.mode; }
		// WGS84 <-> Godot world in the current frame.
		godot::Vector3 wgs84_to_world_h(double latitude, double longitude, double height) const;

		// --- LV95 <-> Godot world ---
		godot::Vector3 lv95_to_world(double east, double north, double elevation) const;
		godot::Vector3 world_to_lv95(godot::Vector3 world_pos) const;

		// --- WGS84 <-> LV95 (static, no origin needed) ---
		static godot::Vector3 wgs84_to_lv95(double latitude, double longitude);
		static godot::Vector3 lv95_to_wgs84(double east, double north);

		// --- WGS84 <-> Godot world (convenience) ---
		godot::Vector3 wgs84_to_world(double latitude, double longitude, double elevation) const;
		godot::Vector3 world_to_wgs84(godot::Vector3 world_pos) const;
	};

}

#endif // S3D_COORDS_H
