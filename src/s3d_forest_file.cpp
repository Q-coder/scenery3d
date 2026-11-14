#include "s3d_forest_file.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

static constexpr uint32_t FOR_MAGIC = 0x31524F46; // 'F','O','R','1'
static constexpr uint32_t FOR_VERSION = 1;

static uint32_t for_read_u32(const uint8_t *p)
{
	uint32_t v;
	std::memcpy(&v, p, 4);
	return v;
}

static float for_read_f32(const uint8_t *p)
{
	float v;
	std::memcpy(&v, p, 4);
	return v;
}

static double for_read_f64(const uint8_t *p)
{
	double v;
	std::memcpy(&v, p, 8);
	return v;
}

bool s3d_parse_forest_file(const std::string &path, std::vector<S3DForestPolygon> &polys,
	std::vector<S3DForestCorridor> &corridors, double &tile_east, double &tile_north)
{
	std::ifstream f(path, std::ios::binary | std::ios::ate);
	if (!f.is_open()) return false;
	const std::streamsize size = f.tellg();
	if (size < 32 || size > (std::streamsize)(64 << 20)) return false;
	f.seekg(0, std::ios::beg);
	std::vector<uint8_t> buf((size_t)size);
	f.read(reinterpret_cast<char *>(buf.data()), size);
	if ((std::streamsize)f.gcount() != size) return false;
	if (for_read_u32(&buf[0]) != FOR_MAGIC || for_read_u32(&buf[4]) != FOR_VERSION) return false;
	const uint32_t count = for_read_u32(&buf[8]);
	tile_east = for_read_f64(&buf[16]);
	tile_north = for_read_f64(&buf[24]);
	size_t p = 32;
	auto need = [&](size_t n) { return p + n <= buf.size(); };
	for (uint32_t i = 0; i < count; i++) {
		if (!need(16)) return false;
		const int kind = buf[p];
		const float density = for_read_f32(&buf[p + 4]);
		const float height = for_read_f32(&buf[p + 8]);
		const uint32_t rings = for_read_u32(&buf[p + 12]);
		p += 16;
		S3DForestPolygon poly;
		poly.kind = kind;
		poly.density = density;
		poly.height_m = height > 1.0f ? height : 20.0f;
		S3DForestCorridor cor;
		cor.half_width = density;
		for (uint32_t r = 0; r < rings; r++) {
			if (!need(4)) return false;
			const uint32_t n = for_read_u32(&buf[p]);
			p += 4;
			if (!need((size_t)n * 8)) return false;
			std::vector<float> ring;
			ring.reserve((size_t)n * 2);
			for (uint32_t k = 0; k < n; k++) {
				ring.push_back(for_read_f32(&buf[p]));
				ring.push_back(for_read_f32(&buf[p + 4]));
				p += 8;
			}
			if (kind == 3) cor.xz = std::move(ring);
			else poly.rings.push_back(std::move(ring));
		}
		if (kind == 3) {
			if (cor.xz.size() >= 4) {
				cor.min_x = cor.max_x = cor.xz[0];
				cor.min_z = cor.max_z = cor.xz[1];
				for (size_t k = 0; k + 1 < cor.xz.size(); k += 2) {
					cor.min_x = std::min(cor.min_x, cor.xz[k]); cor.max_x = std::max(cor.max_x, cor.xz[k]);
					cor.min_z = std::min(cor.min_z, cor.xz[k + 1]); cor.max_z = std::max(cor.max_z, cor.xz[k + 1]);
				}
				corridors.push_back(std::move(cor));
			}
		} else if (!poly.rings.empty() && poly.rings[0].size() >= 6) {
			polys.push_back(std::move(poly));
		}
	}
	return true;
}

bool s3d_point_in_ring(const std::vector<float> &ring, float x, float z)
{
	bool inside = false;
	const size_t n = ring.size() / 2;
	for (size_t i = 0, j = n - 1; i < n; j = i++) {
		const float xi = ring[2 * i], zi = ring[2 * i + 1], xj = ring[2 * j], zj = ring[2 * j + 1];
		if (((zi > z) != (zj > z)) && (x < (xj - xi) * (z - zi) / (zj - zi + 1e-12f) + xi)) inside = !inside;
	}
	return inside;
}

void s3d_fill_landcover_base(int px, std::vector<uint8_t> &rgb)
{
	rgb.resize((size_t)px * px * 3);
	for (size_t i = 0; i < (size_t)px * px; i++) {
		rgb[i * 3] = S3D_LANDCOVER_BASE_RGB[0];
		rgb[i * 3 + 1] = S3D_LANDCOVER_BASE_RGB[1];
		rgb[i * 3 + 2] = S3D_LANDCOVER_BASE_RGB[2];
	}
}

static inline uint32_t lc_hash(uint32_t a, uint32_t b)
{
	uint32_t h = a * 2654435761u ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2));
	h ^= h >> 13; h *= 2246822519u; h ^= h >> 16;
	return h;
}

static inline uint32_t lc_hash_cell(double wx, double wz, double cell, uint32_t salt)
{
	const int32_t cx = (int32_t)std::floor(wx / cell), cz = (int32_t)std::floor(wz / cell);
	return lc_hash(lc_hash((uint32_t)cx, (uint32_t)cz), salt);
}

static inline void lc_set(uint8_t *o, int r, int g, int b)
{
	o[0] = (uint8_t)std::clamp(r, 0, 255);
	o[1] = (uint8_t)std::clamp(g, 0, 255);
	o[2] = (uint8_t)std::clamp(b, 0, 255);
}

// Signed noise in [-amp, amp] from a hash.
static inline int lc_noise(uint32_t h, int amp)
{
	return (int)(h % (uint32_t)(2 * amp + 1)) - amp;
}

// Smooth value noise in [0, 1]: hashed lattice of `cell` metres, smoothstep
// interpolated, so patches have soft edges instead of blocks.
static inline double lc_smooth(double wx, double wz, double cell, uint32_t salt)
{
	const double fx = wx / cell, fz = wz / cell;
	const double ix = std::floor(fx), iz = std::floor(fz);
	double tx = fx - ix, tz = fz - iz;
	tx = tx * tx * (3.0 - 2.0 * tx);
	tz = tz * tz * (3.0 - 2.0 * tz);
	auto at = [&](int dx, int dz) {
		return (double)(lc_hash(lc_hash((uint32_t)((int32_t)ix + dx), (uint32_t)((int32_t)iz + dz)), salt) & 0xFFFF) / 65535.0;
	};
	const double a = at(0, 0) + (at(1, 0) - at(0, 0)) * tx;
	const double b = at(0, 1) + (at(1, 1) - at(0, 1)) * tx;
	return a + (b - a) * tz;
}

// The orientation of a region's parcels and streets: one angle per block of
// `cell` metres, so neighbouring squares share it.
static inline void lc_rotate(double wx, double wz, double cell, uint32_t salt, double &u, double &v)
{
	const uint32_t h = lc_hash_cell(wx, wz, cell, salt);
	const double a = (double)(h & 0xFFFF) / 65535.0 * 3.14159265;
	const double c = std::cos(a), sn = std::sin(a);
	u = wx * c - wz * sn;
	v = wx * sn + wz * c;
}

// Land-cover colour (sRGB) of a pixel at world position (wx, wz) for a
// polygon kind. Everything is a hash of the position, so the pattern is the
// same for every raster resolution and continuous across squares.
//
// The look follows the satellite image rather than the map: farmland is a
// mosaic of parcels in greens, straw and soil browns, towns a speckle of
// roofs and gardens on a street grid, industry large grey roofs, meadows a
// softer green, rock and glacier by altitude's own colours, water a flat
// dark blue-grey. Forests are the dark greens the tree cards stand on.
static void lc_kind_rgb(int kind, double wx, double wz, uint32_t h, uint8_t *o)
{
	switch (kind) {
		case 0: case 1: case 2: {
			int r, g, b;
			if (kind == 0) { r = 58; g = 92; b = 44; }
			else if (kind == 1) { r = 40; g = 70; b = 42; }
			else { r = 48; g = 80; b = 42; }
			const int n = lc_noise(h, 7);
			lc_set(o, r + n, g + n, b + n / 2);
			return;
		}
		case 4: {   // water
			const int n = lc_noise(h, 2);
			lc_set(o, 58 + n, 84 + n, 104 + n);
			return;
		}
		case 5: {   // farmland: irregular strips of parcels, a locally dominant crop
			double u, v;
			lc_rotate(wx, wz, 4096.0, 11u, u, v);
			// Parcel cells 150 x 240 m in the block's orientation, every
			// boundary shifted by up to a third of a cell and the rows
			// staggered per column, so no two parcels are alike.
			const double WU = 150.0, WV = 240.0;
			auto cell_of = [](double x, double w, uint32_t salt) -> int32_t {
				int32_t i = (int32_t)std::floor(x / w);
				auto edge = [&](int32_t k) { return (double)k * w + ((double)(lc_hash((uint32_t)k, salt) & 0xFFFF) / 65535.0 - 0.5) * 0.7 * w; };
				if (x < edge(i)) return i - 1;
				if (x >= edge(i + 1)) return i + 1;
				return i;
			};
			const int32_t iu = cell_of(u, WU, 21u);
			const double v_off = (double)(lc_hash((uint32_t)iu, 22u) & 0xFFFF) / 65535.0 * WV;
			const int32_t iv = cell_of(v + v_off, WV, 23u);
			const uint32_t hp = lc_hash(lc_hash((uint32_t)iu, (uint32_t)iv), 5u);
			// three crop groups: greens, straw, soil; 55 % of the parcels of
			// a 1.2 km block share its dominant group
			static const int palette[3][4][3] = {
				{ { 112, 140, 78 }, { 98, 132, 72 }, { 128, 150, 82 }, { 118, 158, 80 } },
				{ { 186, 172, 116 }, { 170, 158, 102 }, { 198, 184, 130 }, { 160, 150, 98 } },
				{ { 138, 114, 86 }, { 122, 100, 76 }, { 150, 128, 98 }, { 110, 92, 70 } } };
			const uint32_t dominant = lc_hash_cell(wx, wz, 1200.0, 51u) % 3;
			const uint32_t group = (hp % 100) < 55 ? dominant : (hp >> 8) % 3;
			const int *c = palette[group][(hp >> 16) % 4];
			const int bright = lc_noise(hp >> 20, 8);
			const int n = lc_noise(h, 4);
			// faint furrows along the strip, a boundary line where the parcel changes
			const double fv = (v + v_off) / 10.0;
			const int furrow = (fv - std::floor(fv)) < 0.3 ? -5 : 0;
			const double eu = u / WU - std::floor(u / WU), ev = (v + v_off) / WV - std::floor((v + v_off) / WV);
			const int edge = (eu < 0.025 || ev < 0.016) ? -12 : 0;
			lc_set(o, c[0] + bright + n + furrow + edge, c[1] + bright + n + furrow + edge, c[2] + bright / 2 + n / 2 + furrow + edge);
			return;
		}
		case 6: {   // grass, meadow, scrub, parks
			// soft mottling (blocks of one shade read as a checkerboard from above)
			const int m = (int)((lc_smooth(wx, wz, 90.0, 6u) * 0.65 + lc_smooth(wx, wz, 28.0, 61u) * 0.35 - 0.5) * 26.0);
			const int n = lc_noise(h, 4);
			lc_set(o, 120 + m + n, 148 + m + n, 82 + m / 2 + n / 2);
			return;
		}
		case 7: {   // built-up: street grid 70 m, roofs and gardens between
			double u, v;
			lc_rotate(wx, wz, 2048.0, 7u, u, v);
			const double fu = u / 70.0 - std::floor(u / 70.0), fv = v / 70.0 - std::floor(v / 70.0);
			if (fu < 6.0 / 70.0 || fv < 6.0 / 70.0) {
				const int n = lc_noise(h, 3);
				lc_set(o, 118 + n, 116 + n, 112 + n);
				return;
			}
			const uint32_t hc = lc_hash(lc_hash((uint32_t)(int32_t)std::floor(u / 10.0), (uint32_t)(int32_t)std::floor(v / 10.0)), 77u);
			const uint32_t pick = hc % 100;
			const int n = lc_noise(h, 6);
			if (pick < 12) lc_set(o, 130 + n, 106 + n, 96 + n);       // tile roof
			else if (pick < 30) lc_set(o, 122 + n, 120 + n, 116 + n);  // grey roof
			else if (pick < 38) lc_set(o, 98 + n, 96 + n, 94 + n);     // dark roof / asphalt
			else if (pick < 78) lc_set(o, 110 + n, 134 + n, 82 + n);   // garden
			else lc_set(o, 146 + n, 140 + n, 128 + n);                 // yards, pavement
			return;
		}
		case 8: {   // industrial: large roofs 40 m, service roads 120 m
			double u, v;
			lc_rotate(wx, wz, 2048.0, 8u, u, v);
			const double fu = u / 120.0 - std::floor(u / 120.0), fv = v / 120.0 - std::floor(v / 120.0);
			if (fu < 8.0 / 120.0 || fv < 8.0 / 120.0) {
				lc_set(o, 120, 120, 118);
				return;
			}
			const uint32_t hc = lc_hash(lc_hash((uint32_t)(int32_t)std::floor(u / 40.0), (uint32_t)(int32_t)std::floor(v / 40.0)), 88u);
			const int n = lc_noise(h, 3);
			if (hc % 100 < 55) {
				const int lv = 138 + (int)(hc % 40);
				lc_set(o, lv + n, lv + n, lv - 4 + n);
			} else {
				lc_set(o, 156 + n, 156 + n, 150 + n);
			}
			return;
		}
		case 9: {   // bare rock, scree, quarries
			const int m = lc_noise(lc_hash_cell(wx, wz, 30.0, 9u), 10);
			const int n = lc_noise(h, 12);
			lc_set(o, 128 + m + n, 124 + m + n, 118 + m + n);
			return;
		}
		case 10: {  // glacier
			const int m = lc_noise(lc_hash_cell(wx, wz, 20.0, 10u), 5);
			const int n = lc_noise(h, 4);
			lc_set(o, 222 + m + n, 230 + m + n, 236 + n);
			return;
		}
		case 11: {  // sand, beach, mud
			const int n = lc_noise(h, 5);
			lc_set(o, 206 + n, 192 + n, 152 + n);
			return;
		}
		case 12: {  // airfield: worn, mown grass between the runways (gery, ENGM 2026-11-14)
			// Soft patches from 400 m down to 25 m blend a muted green
			// into dry olive and into grey-brown worn ground; no parcels,
			// no roofs. Faint mowing stripes in the block's orientation.
			const double big = lc_smooth(wx, wz, 400.0, 121u);
			const double mid = lc_smooth(wx, wz, 110.0, 122u);
			const double fine = lc_smooth(wx, wz, 25.0, 123u);
			const double dry = std::clamp((big * 0.5 + mid * 0.35 + fine * 0.15 - 0.38) * 2.6, 0.0, 1.0);
			const double worn = std::clamp((mid * 0.6 + fine * 0.4 - 0.62) * 4.0, 0.0, 1.0) * big;
			double r = 104.0 + (128.0 - 104.0) * dry, g = 128.0 + (130.0 - 128.0) * dry, b = 76.0 + (88.0 - 76.0) * dry;
			r += (124.0 - r) * worn; g += (118.0 - g) * worn; b += (102.0 - b) * worn;
			double u, v;
			lc_rotate(wx, wz, 4096.0, 12u, u, v);
			const double st = v / 24.0 - std::floor(v / 24.0);
			const int stripe = st < 0.5 ? 2 : -2;
			const int n = lc_noise(h, 4);
			lc_set(o, (int)r + n + stripe, (int)g + n + stripe, (int)b + n / 2 + stripe);
			return;
		}
		default:
			lc_set(o, S3D_LANDCOVER_BASE_RGB[0], S3D_LANDCOVER_BASE_RGB[1], S3D_LANDCOVER_BASE_RGB[2]);
			return;
	}
}

// Paint order: the wide land-use classes first, forests over them, water last.
static int lc_rank(int kind)
{
	switch (kind) {
		case 12: return 0;   // the aerodrome boundary: everything mapped inside it paints over
		case 5: return 1; case 6: return 2; case 7: return 3; case 8: return 4;
		case 9: return 5; case 10: return 6; case 11: return 7;
		case 0: case 1: case 2: return 8;
		case 4: return 9;
		default: return -1;
	}
}

void s3d_rasterise_landcover(const std::vector<S3DForestPolygon> &polys,
	const std::vector<S3DForestCorridor> &corridors, int px, float square_m,
	double tile_east, double tile_north, std::vector<uint8_t> &rgb)
{
	s3d_fill_landcover_base(px, rgb);
	if (px < 1 || square_m <= 0.0f) return;
	const float m_per_px = square_m / (float)px;
	// Column of an east offset x: column 0 is the east edge.
	auto col_of = [&](float x) { return px - 1 - (int)std::floor(x / m_per_px); };

	std::vector<size_t> order;
	order.reserve(polys.size());
	for (size_t i = 0; i < polys.size(); i++) if (lc_rank(polys[i].kind) >= 0) order.push_back(i);
	std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return lc_rank(polys[a].kind) < lc_rank(polys[b].kind); });

	std::vector<float> xs;
	for (size_t oi : order) {
		const S3DForestPolygon &poly = polys[oi];
		float minz = 1e30f, maxz = -1e30f;
		for (const auto &ring : poly.rings) {
			for (size_t k = 1; k < ring.size(); k += 2) { minz = std::min(minz, ring[k]); maxz = std::max(maxz, ring[k]); }
		}
		const int r0 = std::max(0, (int)std::floor(minz / m_per_px));
		const int r1 = std::min(px - 1, (int)std::floor(maxz / m_per_px));
		for (int row = r0; row <= r1; row++) {
			// Scanline through the pixel-centre northing; even-odd across
			// every ring of the polygon so holes come out as base.
			const float z = ((float)row + 0.5f) * m_per_px;
			xs.clear();
			for (const auto &ring : poly.rings) {
				const size_t n = ring.size() / 2;
				for (size_t i = 0, j = n - 1; i < n; j = i++) {
					const float xi = ring[2 * i], zi = ring[2 * i + 1], xj = ring[2 * j], zj = ring[2 * j + 1];
					if ((zi > z) != (zj > z)) xs.push_back((xj - xi) * (z - zi) / (zj - zi + 1e-12f) + xi);
				}
			}
			if (xs.size() < 2) continue;
			std::sort(xs.begin(), xs.end());
			for (size_t k = 0; k + 1 < xs.size(); k += 2) {
				const float xa = std::max(0.0f, xs[k]), xb = std::min(square_m, xs[k + 1]);
				if (xb <= xa) continue;
				// Pixel centres between xa and xb.
				const int ca = (int)std::floor(xa / m_per_px + 0.5f);
				const int cb = (int)std::floor(xb / m_per_px - 0.5f);
				for (int c = ca; c <= cb; c++) {
					const int col = px - 1 - c;
					if (col < 0 || col >= px) continue;
					uint8_t *o = &rgb[((size_t)row * px + col) * 3];
					const double wx = tile_east + ((double)c + 0.5) * m_per_px;
					const double wz = tile_north + (double)z;
					lc_kind_rgb(poly.kind, wx, wz, lc_hash((uint32_t)(int32_t)std::floor(wx), (uint32_t)(int32_t)std::floor(wz)), o);
				}
			}
		}
	}

	if (px < 64 || corridors.empty()) return;
	// The main roads (tertiary and up, half width >= 5.5 m) as faint grey
	// lines blended into the ground: the network a pilot reads the ground
	// by, without turning a village into a street map. Lanes, tracks and
	// railways stay in the ground colour.
	for (const S3DForestCorridor &c : corridors) {
		if (c.half_width < 5.5f) continue;
		const float r = c.half_width;
		const int cmin = std::max(0, col_of(std::min(square_m, c.max_x + r)));
		const int cmax = std::min(px - 1, col_of(std::max(0.0f, c.min_x - r)));
		const int rmin = std::max(0, (int)std::floor((c.min_z - r) / m_per_px));
		const int rmax = std::min(px - 1, (int)std::floor((c.max_z + r) / m_per_px));
		for (int row = rmin; row <= rmax; row++) {
			const float z = ((float)row + 0.5f) * m_per_px;
			for (int col = cmin; col <= cmax; col++) {
				const float x = ((float)(px - 1 - col) + 0.5f) * m_per_px;
				bool hit = false;
				for (size_t k = 0; k + 3 < c.xz.size() && !hit; k += 2) {
					const float ax = c.xz[k], az = c.xz[k + 1], bx = c.xz[k + 2], bz = c.xz[k + 3];
					const float vx = bx - ax, vz = bz - az;
					const float len2 = vx * vx + vz * vz;
					float u = len2 > 1e-6f ? ((x - ax) * vx + (z - az) * vz) / len2 : 0.0f;
					u = std::clamp(u, 0.0f, 1.0f);
					const float dx = ax + u * vx - x, dz = az + u * vz - z;
					hit = dx * dx + dz * dz <= r * r;
				}
				if (!hit) continue;
				uint8_t *o = &rgb[((size_t)row * px + col) * 3];
				const int n = lc_noise(lc_hash((uint32_t)col, (uint32_t)row), 3);
				lc_set(o, (o[0] * 2 + (112 + n) * 3) / 5, (o[1] * 2 + (112 + n) * 3) / 5, (o[2] * 2 + (104 + n) * 3) / 5);
			}
		}
	}
}

bool s3d_landcover_raster(const std::vector<std::string> &candidates, int px, float square_m, std::vector<uint8_t> &rgb)
{
	for (const std::string &path : candidates) {
		std::vector<S3DForestPolygon> polys;
		std::vector<S3DForestCorridor> corridors;
		double e = 0, n = 0;
		if (!s3d_parse_forest_file(path, polys, corridors, e, n)) continue;
		s3d_rasterise_landcover(polys, corridors, px, square_m, e, n, rgb);
		return true;
	}
	return false;
}
