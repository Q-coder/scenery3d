// s3d_frame.h — where Scenery3D geometry lives in the Godot world.
//
// Scenery3D's tile grid is a projected CRS (LV95 by default): tile meshes,
// GLB tiles and tree instances are all authored as FLAT offsets from some
// grid anchor point (local x = anchor_e - E  [+X = West], local y = metres
// ASL, local z = N - anchor_n [+Z = North]). S3DFrame decides how such a
// node is placed in the world:
//
//   FLAT (mode 0, legacy): the world IS the flat frame around
//        (origin_east, origin_north): node transform = translation only,
//        world y = ASL everywhere. Curvature is ignored.
//   ECEF (mode 1, design §9.5 step C): the world is WGS84 ECEF metres. A node
//        is placed with the local WUN frame (+X West, +Y Up, +Z North) at its
//        tile CENTRE on the ellipsoid, times the 2x2 Jacobian of the CRS
//        inverse there (grid metres -> true metres: carries the projection's
//        scale and grid convergence), times the translation that puts the
//        tile centre at the local origin. Meshes stay tile-local; a
//        curvature sag term (see sag()) is applied per vertex where the tile
//        is big enough to matter (1.25 m at an 8 km chunk corner, 2 cm at a
//        1 km tile corner). Neighbouring tiles compute their shared edge from
//        different centres; the mismatch is second order (mm at 1 km).
//
// Header-only so every manager holds a copy by value; Scenery3D fans the
// mode/origin out to its children.
#ifndef S3D_FRAME_H
#define S3D_FRAME_H

#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <cmath>
#include "s3d_proj.h"

namespace s3d
{

	struct S3DFrame
	{
		enum Mode { FLAT = 0, ECEF = 1 };

		int mode = FLAT;
		double origin_east = 2600000.0;  // FLAT: grid point at world (0, *, 0)
		double origin_north = 1200000.0;
		proj::Crs crs;                   // CRS of the tile grid (LV95 default)
		static constexpr double R_MEAN = 6371000.0;

		bool ecef() const { return mode == ECEF; }

		// Grid E/N under a world position.
		proj::EN world_to_grid(const godot::Vector3 &w) const
		{
			if (mode == FLAT) {
				return proj::EN{origin_east - (double)w.x, (double)w.z + origin_north};
			}
			const proj::LatLon ll = proj::ecef_to_geodetic(proj::WGS84, proj::XYZ{(double)w.x, (double)w.y, (double)w.z});
			return crs.forward(ll.lat, ll.lon);
		}

		// Height (ASL, treated as ellipsoidal in ECEF mode) of a world position.
		double world_height(const godot::Vector3 &w) const
		{
			if (mode == FLAT) return (double)w.y;
			double h = 0.0;
			proj::ecef_to_geodetic(proj::WGS84, proj::XYZ{(double)w.x, (double)w.y, (double)w.z}, &h);
			return h;
		}

		// World position of a grid point at height h.
		godot::Vector3 grid_to_world(double e, double n, double h) const
		{
			if (mode == FLAT) {
				return godot::Vector3(origin_east - e, h, n - origin_north);
			}
			const proj::LatLon ll = crs.inverse(e, n);
			const proj::XYZ p = proj::geodetic_to_ecef(proj::WGS84, ll.lat, ll.lon, h);
			return godot::Vector3(p.x, p.y, p.z);
		}

		// Local WUN frame (+X West, +Y Up, +Z North) on the ellipsoid (h = 0)
		// at a grid point. ECEF mode only (FLAT returns a translation).
		godot::Transform3D wun_at(double e, double n) const
		{
			if (mode == FLAT) {
				return godot::Transform3D(godot::Basis(), grid_to_world(e, n, 0.0));
			}
			const proj::LatLon ll = crs.inverse(e, n);
			const double lat = proj::deg2rad(ll.lat), lon = proj::deg2rad(ll.lon);
			const double sl = std::sin(lat), cl = std::cos(lat), so = std::sin(lon), co = std::cos(lon);
			const godot::Vector3 east(-so, co, 0.0);
			const godot::Vector3 north(-sl * co, -sl * so, cl);
			const godot::Vector3 up(cl * co, cl * so, sl);
			godot::Basis b;
			b.set_column(0, -east);
			b.set_column(1, up);
			b.set_column(2, north);
			const proj::XYZ o = proj::geodetic_to_ecef(proj::WGS84, ll.lat, ll.lon, 0.0);
			return godot::Transform3D(b, godot::Vector3(o.x, o.y, o.z));
		}

		// Basis mapping tile-local GRID metres (x = -dE, y = up, z = dN) to
		// WUN-local TRUE metres at the tile centre: the CRS's scale factor and
		// grid convergence, evaluated numerically. Identity in FLAT mode.
		godot::Basis grid_basis_at(double center_e, double center_n) const
		{
			if (mode == FLAT) return godot::Basis();
			const godot::Transform3D wun = wun_at(center_e, center_n);
			const godot::Basis inv = wun.basis.transposed();
			const double d = 100.0;
			godot::Vector3 ue = inv.xform(grid_to_world(center_e + d, center_n, 0.0) - wun.origin) / d;
			godot::Vector3 un = inv.xform(grid_to_world(center_e, center_n + d, 0.0) - wun.origin) / d;
			ue.y = 0.0;
			un.y = 0.0;
			godot::Basis a;
			a.set_column(0, -ue);
			a.set_column(1, godot::Vector3(0, 1, 0));
			a.set_column(2, un);
			return a;
		}

		// Transform for a node whose local coordinates are flat offsets from
		// grid anchor (anchor_e, anchor_n) and whose content is centred on
		// (center_e, center_n); y_offset is applied along local up.
		godot::Transform3D place(double anchor_e, double anchor_n, double center_e, double center_n, double y_offset) const
		{
			if (mode == FLAT) {
				return godot::Transform3D(godot::Basis(), godot::Vector3(origin_east - anchor_e, y_offset, anchor_n - origin_north));
			}
			const godot::Transform3D wun = wun_at(center_e, center_n);
			const godot::Basis a = grid_basis_at(center_e, center_n);
			// Local coordinates of the centre point.
			const godot::Vector3 lc(anchor_e - center_e, 0.0, center_n - anchor_n);
			const godot::Transform3D t(a, -a.xform(lc) + godot::Vector3(0.0, y_offset, 0.0));
			return wun * t;
		}

		// Curvature drop of the ellipsoid below the tangent plane at a tile
		// centre, for a vertex (dx, dz) metres from that centre.
		double sag(double dx, double dz) const
		{
			return mode == ECEF ? (dx * dx + dz * dz) / (2.0 * R_MEAN) : 0.0;
		}
	};

}

#endif // S3D_FRAME_H
