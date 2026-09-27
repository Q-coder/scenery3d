// s3d_exclusion.h — exclusion zones of higher-priority scenery packs.
//
// A bespoke pack (a hand-made airfield) can declare zones in which the
// buildings and trees of lower-priority packs must not appear. The host
// passes them as an Array of Dictionaries:
//   {"kinds": "buildings,vegetation,objects" | Array[String] | "all",
//    "min_dir": int,                       // first path index the zone applies to
//    "circle": [e, n, r]}                  // LV95 metres, or
//   {"polygon": PackedVector2Array | Array[Vector2] (LV95 e, n)}
// `min_dir` is the index into the manager's path list (priority order) of
// the first lower-priority pack: paths before it belong to the pack itself
// or to packs above it and are never filtered.
#pragma once

#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace s3d {

	struct S3DExclusionZone {
		int min_dir = 0;
		bool circle = false;
		double e = 0.0, n = 0.0, r = 0.0;
		std::vector<std::pair<double, double>> poly;
		double min_e = 0.0, max_e = 0.0, min_n = 0.0, max_n = 0.0;

		bool contains(double pe, double pn) const
		{
			if (circle) {
				const double de = pe - e, dn = pn - n;
				return de * de + dn * dn <= r * r;
			}
			if (pe < min_e || pe > max_e || pn < min_n || pn > max_n) return false;
			bool inside = false;
			for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
				const double xi = poly[i].first, yi = poly[i].second;
				const double xj = poly[j].first, yj = poly[j].second;
				if ((yi > pn) != (yj > pn) && pe < (xj - xi) * (pn - yi) / (yj - yi + 1e-12) + xi) inside = !inside;
			}
			return inside;
		}
	};

	struct S3DExclusionZones {
		std::vector<S3DExclusionZone> zones;

		bool empty() const { return zones.empty(); }

		// Keep the zones whose kinds include `kind` (or "all").
		void set_from_array(const godot::Array &arr, const char *kind)
		{
			zones.clear();
			const std::string want(kind);
			for (int i = 0; i < arr.size(); i++) {
				if (arr[i].get_type() != godot::Variant::DICTIONARY) continue;
				const godot::Dictionary d = arr[i];
				bool applies = false;
				const godot::Variant kinds = d.get("kinds", "all");
				if (kinds.get_type() == godot::Variant::ARRAY) {
					const godot::Array ka = kinds;
					for (int k = 0; k < ka.size(); k++) {
						const std::string s = godot::String(ka[k]).utf8().get_data();
						if (s == want || s == "all") applies = true;
					}
				} else {
					const std::string s = godot::String(kinds).utf8().get_data();
					applies = s == "all" || s.find(want) != std::string::npos;
				}
				if (!applies) continue;
				S3DExclusionZone z;
				z.min_dir = (int)d.get("min_dir", 0);
				if (d.has("circle")) {
					const godot::Array c = d["circle"];
					if (c.size() < 3) continue;
					z.circle = true;
					z.e = (double)c[0];
					z.n = (double)c[1];
					z.r = (double)c[2];
				} else if (d.has("polygon")) {
					const godot::Variant pv = d["polygon"];
					godot::PackedVector2Array pts;
					if (pv.get_type() == godot::Variant::PACKED_VECTOR2_ARRAY) {
						pts = pv;
					} else if (pv.get_type() == godot::Variant::ARRAY) {
						const godot::Array a = pv;
						for (int k = 0; k < a.size(); k++) pts.push_back(godot::Vector2(a[k]));
					}
					if (pts.size() < 3) continue;
					z.min_e = z.min_n = 1e300;
					z.max_e = z.max_n = -1e300;
					for (int k = 0; k < pts.size(); k++) {
						z.poly.push_back({(double)pts[k].x, (double)pts[k].y});
						z.min_e = std::fmin(z.min_e, pts[k].x);
						z.max_e = std::fmax(z.max_e, pts[k].x);
						z.min_n = std::fmin(z.min_n, pts[k].y);
						z.max_n = std::fmax(z.max_n, pts[k].y);
					}
				} else {
					continue;
				}
				zones.push_back(z);
			}
		}

		bool contains(double e, double n, int dir_index) const
		{
			for (const S3DExclusionZone &z : zones) {
				if (dir_index >= z.min_dir && z.contains(e, n)) return true;
			}
			return false;
		}
	};

}
