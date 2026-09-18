#include "s3d_coords.h"
#include "s3d_proj.h"
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
	ClassDB::bind_method(D_METHOD("set_frame_mode", "mode"), &S3DCoords::set_frame_mode);
	ClassDB::bind_method(D_METHOD("get_frame_mode"), &S3DCoords::get_frame_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "frame_mode"), "set_frame_mode", "get_frame_mode");
	ClassDB::bind_method(D_METHOD("wgs84_to_world_h", "latitude", "longitude", "height"), &S3DCoords::wgs84_to_world_h);

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
	frame.origin_east = p_east;
}

double S3DCoords::get_origin_east() const
{
	return origin_east;
}

void S3DCoords::set_origin_north(double p_north)
{
	origin_north = p_north;
	frame.origin_north = p_north;
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
	// Flat: +X = West, +Z = North (provpilot convention); ECEF: on the ellipsoid.
	return frame.grid_to_world(east, north, elevation);
}

Vector3 S3DCoords::world_to_lv95(Vector3 world_pos) const
{
	// Returns (East, North, Elevation).
	const proj::EN en = frame.world_to_grid(world_pos);
	return Vector3(en.e, en.n, frame.world_height(world_pos));
}

void S3DCoords::set_frame_mode(int p_mode)
{
	frame.mode = (p_mode == S3DFrame::ECEF) ? S3DFrame::ECEF : S3DFrame::FLAT;
}

Vector3 S3DCoords::wgs84_to_world_h(double latitude, double longitude, double height) const
{
	const proj::EN en = proj::wgs84_to_lv95(latitude, longitude);
	return frame.grid_to_world(en.e, en.n, height);
}

// ---- WGS84 <-> LV95 (rigorous, see s3d_proj.h) ----
// Swiss oblique Mercator on Bessel 1841 + CH1903+/WGS84 datum shift: matches
// pyproj EPSG:4326 <-> EPSG:2056 (the bake pipeline) to the millimetre
// anywhere in Europe. The former swisstopo approximation polynomial drifted
// by kilometres outside Switzerland (32 km at Kirkenes).

Vector3 S3DCoords::wgs84_to_lv95(double latitude, double longitude)
{
	const proj::EN en = proj::wgs84_to_lv95(latitude, longitude);
	// Returns (East, North, 0).
	return Vector3(en.e, en.n, 0.0);
}

Vector3 S3DCoords::lv95_to_wgs84(double east, double north)
{
	const proj::LatLon ll = proj::lv95_to_wgs84(east, north);
	// Returns (latitude, longitude, 0).
	return Vector3(ll.lat, ll.lon, 0.0);
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
