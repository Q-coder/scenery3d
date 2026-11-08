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

// Forest greens by kind (sRGB): broadleaf a warm dark green, conifer darker
// and cooler, mixed between; all clearly darker than the base so a wood reads
// as a wood against fields from any distance, as on an orthophoto.
static void lc_forest_rgb(int kind, uint32_t h, uint8_t *out)
{
	int r, g, b;
	switch (kind) {
		case 0: r = 58; g = 92; b = 44; break;
		case 1: r = 40; g = 70; b = 42; break;
		default: r = 48; g = 80; b = 42; break;
	}
	// ±7 of luminance-correlated noise, the same seed per pixel across kinds.
	const int n = (int)(h & 15) - 7;
	out[0] = (uint8_t)std::clamp(r + n, 0, 255);
	out[1] = (uint8_t)std::clamp(g + n, 0, 255);
	out[2] = (uint8_t)std::clamp(b + n / 2, 0, 255);
}

void s3d_rasterise_landcover(const std::vector<S3DForestPolygon> &polys,
	const std::vector<S3DForestCorridor> &corridors, int px, float square_m, std::vector<uint8_t> &rgb)
{
	s3d_fill_landcover_base(px, rgb);
	if (px < 1 || square_m <= 0.0f) return;
	const float m_per_px = square_m / (float)px;
	// Column of an east offset x: column 0 is the east edge.
	auto col_of = [&](float x) { return px - 1 - (int)std::floor(x / m_per_px); };

	std::vector<float> xs;
	for (const S3DForestPolygon &poly : polys) {
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
					lc_forest_rgb(poly.kind, lc_hash((uint32_t)col, (uint32_t)row), o);
				}
			}
		}
	}

	if (px < 64 || corridors.empty()) return;
	// Clear the corridors to base: a road through a wood keeps its gap in the
	// texture as it does in the tree cards.
	for (const S3DForestCorridor &c : corridors) {
		const float r = c.half_width + 1.0f;
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
				o[0] = S3D_LANDCOVER_BASE_RGB[0]; o[1] = S3D_LANDCOVER_BASE_RGB[1]; o[2] = S3D_LANDCOVER_BASE_RGB[2];
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
		s3d_rasterise_landcover(polys, corridors, px, square_m, rgb);
		return true;
	}
	return false;
}
