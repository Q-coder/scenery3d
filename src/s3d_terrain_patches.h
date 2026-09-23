#pragma once
// Terrain patches applied at tile load (docs/SCENERY_EDITOR.md, "apply the
// design at load time"): the scenery editor's plateaus and road / track
// corridors, written into a tile's height image the moment it is decoded,
// exactly as scripts/build_world_roads.py --corridor writes them into the
// baked tiles. Pixel layout of a tile: (0,0) is the south-east corner,
// columns run east -> west, rows south -> north (S3DTile::set_placement
// anchors the tile at (ei + 1) * size, ni * size).

#include <array>
#include <cmath>
#include <utility>
#include <vector>

struct S3DPlateau {
	std::vector<std::pair<double, double>> poly;   // LV95 (e, n)
	double h = 0.0;                                // metres ASL
	double e0 = 0, e1 = 0, n0 = 0, n1 = 0;         // bounding box
};

struct S3DCorridor {
	std::vector<std::array<double, 3>> st;         // stations (e, n, h)
	double reach = 0.0;                            // metres either side of the line
	double half_width = 0.0;                       // the ribbon's half width (blend weights start at its edge)
	double e0 = 0, e1 = 0, n0 = 0, n1 = 0;         // bounding box (stations + reach)
};

struct S3DTerrainPatches {
	std::vector<S3DPlateau> plateaus;
	std::vector<S3DCorridor> corridors;
	// Airport plateaus (runway strips, taxiways as corridors; aprons as
	// polygons): a design corridor never changes a pixel inside them.
	std::vector<S3DPlateau> protect_polys;
	std::vector<S3DCorridor> protect_corridors;

	bool empty() const { return plateaus.empty() && corridors.empty(); }


	static bool point_in_polygon(double x, double y, const std::vector<std::pair<double, double>> &poly) {
		bool inside = false;
		size_t j = poly.size() - 1;
		for (size_t i = 0; i < poly.size(); j = i++) {
			double xi = poly[i].first, yi = poly[i].second;
			double xj = poly[j].first, yj = poly[j].second;
			if ((yi > y) != (yj > y)) {
				double d = (yj - yi);
				double x_at = (xj - xi) * (y - yi) / (d == 0.0 ? 1e-12 : d) + xi;
				if (x < x_at) inside = !inside;
			}
		}
		return inside;
	}

	// Height of the nearest corridor segment within reach, or NaN; the
	// distance to it in *dist when given.
	static double corridor_height(const S3DCorridor &c, double e, double n, double *dist = nullptr) {
		double best_d2 = c.reach * c.reach;
		double best_h = NAN;
		for (size_t i = 0; i + 1 < c.st.size(); i++) {
			double x0 = c.st[i][0], y0 = c.st[i][1], x1 = c.st[i + 1][0], y1 = c.st[i + 1][1];
			double dx = x1 - x0, dy = y1 - y0;
			double l2 = dx * dx + dy * dy;
			double t = l2 > 1e-9 ? ((e - x0) * dx + (n - y0) * dy) / l2 : 0.0;
			t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
			double px = x0 + dx * t, py = y0 + dy * t;
			double d2 = (e - px) * (e - px) + (n - py) * (n - py);
			if (d2 < best_d2) {
				best_d2 = d2;
				best_h = c.st[i][2] + (c.st[i + 1][2] - c.st[i][2]) * t;
			}
		}
		if (dist) *dist = std::sqrt(best_d2);
		return best_h;
	}

	bool protected_at(double e, double n) const {
		for (const auto &p : protect_polys) {
			if (e < p.e0 || e > p.e1 || n < p.n0 || n > p.n1) continue;
			if (point_in_polygon(e, n, p.poly)) return true;
		}
		for (const auto &c : protect_corridors) {
			if (e < c.e0 || e > c.e1 || n < c.n0 || n > c.n1) continue;
			if (!std::isnan(corridor_height(c, e, n))) return true;
		}
		return false;
	}

	// Write the patches into a tile's float heights (w x h pixels anchored at
	// the south-east corner (anchor_e, anchor_n), size_m across).
	// Returns the number of pixels changed.
	int apply(float *data, int w, int h, double anchor_e, double anchor_n, double size_m) const {
		if (empty() || w < 2 || h < 2) return 0;
		const double te0 = anchor_e - size_m, te1 = anchor_e, tn0 = anchor_n, tn1 = anchor_n + size_m;
		std::vector<const S3DPlateau *> pl;
		for (const auto &p : plateaus)
			if (p.e1 >= te0 && p.e0 <= te1 && p.n1 >= tn0 && p.n0 <= tn1) pl.push_back(&p);
		std::vector<const S3DCorridor *> co;
		for (const auto &c : corridors)
			if (c.e1 >= te0 && c.e0 <= te1 && c.n1 >= tn0 && c.n0 <= tn1) co.push_back(&c);
		if (pl.empty() && co.empty()) return 0;
		const double step = size_m / (double)(w - 1);
		int changed = 0;
		for (int py = 0; py < h; py++) {
			const double n = anchor_n + py * step;
			for (int px = 0; px < w; px++) {
				const double e = anchor_e - px * step;
				float &v = data[py * w + px];
				// corridors first (where they overlap, the heights blend by
				// 1 / (edge distance + 3 m)^2 as in build_world_roads.py),
				// plateaus win over corridors (a road on a plateau lies on it);
				// airport plateaus are never touched by a corridor
				double sw = 0.0, swh = 0.0;
				for (const S3DCorridor *c : co) {
					if (e < c->e0 || e > c->e1 || n < c->n0 || n > c->n1) continue;
					double d = 0.0;
					double hh = corridor_height(*c, e, n, &d);
					if (std::isnan(hh)) continue;
					double ed = d - c->half_width;
					double w = 1.0 / ((ed < 0.0 ? 0.0 : ed) + 3.0) / ((ed < 0.0 ? 0.0 : ed) + 3.0);
					sw += w; swh += w * hh;
				}
				if (sw > 0.0 && !protected_at(e, n)) { v = (float)(swh / sw); changed++; }
				for (const S3DPlateau *p : pl) {
					if (e < p->e0 || e > p->e1 || n < p->n0 || n > p->n1) continue;
					if (point_in_polygon(e, n, p->poly)) { v = (float)p->h; changed++; }
				}
			}
		}
		return changed;
	}
};

