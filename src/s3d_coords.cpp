#include "s3d_coords.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

namespace s3d {

S3DCoords::S3DCoords()
{
}

S3DCoords::~S3DCoords()
{
}

void S3DCoords::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DCoords::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DCoords::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DCoords::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DCoords::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_origin_from_wgs84", "latitude", "longitude"), &S3DCoords::set_origin_from_wgs84);

	ClassDB::bind_method(D_METHOD("lv95_to_world", "east", "north", "elevation"), &S3DCoords::lv95_to_world);
	ClassDB::bind_method(D_METHOD("world_to_lv95", "world_pos"), &S3DCoords::world_to_lv95);

	ClassDB::bind_static_method("S3DCoords", D_METHOD("wgs84_to_lv95", "latitude", "longitude"), &S3DCoords::wgs84_to_lv95);
	ClassDB::bind_static_method("S3DCoords", D_METHOD("lv95_to_wgs84", "east", "north"), &S3DCoords::lv95_to_wgs84);

	ClassDB::bind_method(D_METHOD("wgs84_to_world", "latitude", "longitude", "elevation"), &S3DCoords::wgs84_to_world);
	ClassDB::bind_method(D_METHOD("world_to_wgs84", "world_pos"), &S3DCoords::world_to_wgs84);
}

void S3DCoords::set_origin_east(double p_east)
{
	origin_east = p_east;
}

double S3DCoords::get_origin_east() const
{
	return origin_east;
}

void S3DCoords::set_origin_north(double p_north)
{
	origin_north = p_north;
}

double S3DCoords::get_origin_north() const
{
	return origin_north;
}

void S3DCoords::set_origin_from_wgs84(double latitude, double longitude)
{
	Vector3 lv = wgs84_to_lv95(latitude, longitude);
	origin_east = lv.x;
	origin_north = lv.y;
}

// ---- LV95 <-> Godot world ----

Vector3 S3DCoords::lv95_to_world(double east, double north, double elevation) const
{
	// +X = West, +Z = North (matching provpilot convention)
	return Vector3(origin_east - east, elevation, north - origin_north);
}

Vector3 S3DCoords::world_to_lv95(Vector3 world_pos) const
{
	// Returns (East, North, Elevation).
	return Vector3(origin_east - world_pos.x, world_pos.z + origin_north, world_pos.y);
}

// ---- WGS84 <-> LV95 (SwissTopo approximate formulas) ----
// Reference: swisstopo, "Approximate formulas for the transformation between
// the Swiss projection system and WGS84" (2016).

Vector3 S3DCoords::wgs84_to_lv95(double latitude, double longitude)
{
	// Auxiliary values: convert to arc-seconds and shift from Bern.
	double phi_prime = (latitude - 46.9524058333) * 10000.0 / 3600.0;
	double lambda_prime = (longitude - 7.4395833333) * 10000.0 / 3600.0;

	double lp = lambda_prime;
	double pp = phi_prime;

	double east = 2600072.37
		+ 211455.93 * lp
		- 10938.51 * lp * pp
		- 0.36 * lp * pp * pp
		- 44.54 * lp * lp * lp;

	double north = 1200147.07
		+ 308807.95 * pp
		+ 3745.25 * lp * lp
		+ 76.63 * pp * pp
		- 194.56 * lp * lp * pp
		+ 119.79 * pp * pp * pp;

	// Returns (East, North, 0).
	return Vector3(east, north, 0.0);
}

Vector3 S3DCoords::lv95_to_wgs84(double east, double north)
{
	// Auxiliary values.
	double y_prime = (east - 2600000.0) / 1000000.0;
	double x_prime = (north - 1200000.0) / 1000000.0;

	double yp = y_prime;
	double xp = x_prime;

	// Longitude and latitude in 10000" units.
	double lambda_s = 2.6779094
		+ 4.728982 * yp
		+ 0.791484 * yp * xp
		+ 0.1306 * yp * xp * xp
		- 0.0436 * yp * yp * yp;

	double phi_s = 16.9023892
		+ 3.238272 * xp
		- 0.270978 * yp * yp
		- 0.002528 * xp * xp
		- 0.0447 * yp * yp * xp
		- 0.0140 * xp * xp * xp;

	// Convert from 10000" to degrees: value * 100 / 36.
	double longitude = lambda_s * 100.0 / 36.0;
	double latitude = phi_s * 100.0 / 36.0;

	// Returns (latitude, longitude, 0).
	return Vector3(latitude, longitude, 0.0);
}

// ---- WGS84 <-> Godot world (convenience) ----

Vector3 S3DCoords::wgs84_to_world(double latitude, double longitude, double elevation) const
{
	Vector3 lv = wgs84_to_lv95(latitude, longitude);
	return lv95_to_world(lv.x, lv.y, elevation);
}

Vector3 S3DCoords::world_to_wgs84(Vector3 world_pos) const
{
	Vector3 lv = world_to_lv95(world_pos);
	Vector3 geo = lv95_to_wgs84(lv.x, lv.y);
	// Returns (latitude, longitude, elevation).
	return Vector3(geo.x, geo.y, world_pos.y);
}

} // namespace s3d
