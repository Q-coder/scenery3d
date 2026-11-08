#pragma once

// forest_{E}_{N}.bin (FOR1, scripts/build_world_forests.py): the OSM forest
// polygons and road / rail corridors of one 1024 m square, shared by the
// vegetation manager (fills them with tree cards near the aircraft) and the
// tile manager (paints them into the ground texture of every tile without an
// orthophoto, so forests stay recognisable to the horizon — design §14.6).
//
// File (little-endian): "FOR1", u32 1, u32 polygon count, u32 0, f64 tile
// east, f64 tile north (SW corner, LV95-like metres); polygon: u8 kind
// (0 broadleaf, 1 conifer, 2 mixed, 3 corridor, 4 water, 5 farmland, 6 grass,
// 7 built-up, 8 industrial, 9 bare rock, 10 glacier, 11 sand), u8[3], f32
// density (trees per m2; corridor: half width m; land cover 0), f32 height m,
// u32 ring count; ring: u32 n, n x (f32 dx east, f32 dz north) from the SW
// corner. Only kinds 0-2 get trees.

#include <cstdint>
#include <string>
#include <vector>

struct S3DForestCorridor {
	std::vector<float> xz;   // dx, dz pairs along the centre line
	float half_width = 3.0f;
	float min_x = 0, min_z = 0, max_x = 0, max_z = 0;
};

struct S3DForestPolygon {
	int kind = 2;            // 0 broadleaf, 1 conifer, 2 mixed; land cover 4 water, 5 farmland, 6 grass, 7 built-up, 8 industrial, 9 rock, 10 glacier, 11 sand
	float density = 1.0f / 36.0f;   // trees per m2
	float height_m = 20.0f;
	std::vector<std::vector<float>> rings;   // outer first, then holes; dx, dz pairs
};

bool s3d_parse_forest_file(const std::string &path, std::vector<S3DForestPolygon> &polys,
	std::vector<S3DForestCorridor> &corridors, double &tile_east, double &tile_north);

bool s3d_point_in_ring(const std::vector<float> &ring, float x, float z);

// The plain ground colour of a tile without an orthophoto (sRGB bytes), the
// same as the tile manager's shared material albedo (0.45, 0.55, 0.35).
constexpr uint8_t S3D_LANDCOVER_BASE_RGB[3] = { 115, 140, 89 };

// px x px RGB8 land-cover raster of a square, in orthophoto orientation
// (column 0 = east edge, row 0 = south edge; square_m metres per side): base
// colour where no polygon; the polygons by kind — forests dark green, water,
// farmland parcels, grass, built-up roofs on a street grid, industry, rock,
// glacier, sand — as position-hashed patterns continuous across squares
// (tile_east / tile_north place the square in the world; holes stay base);
// the corridors as grey road lines when the raster is fine enough (px >= 64).
void s3d_rasterise_landcover(const std::vector<S3DForestPolygon> &polys,
	const std::vector<S3DForestCorridor> &corridors, int px, float square_m,
	double tile_east, double tile_north, std::vector<uint8_t> &rgb);

// px x px of the base colour.
void s3d_fill_landcover_base(int px, std::vector<uint8_t> &rgb);

// The first candidate forest file that parses, rasterised at px; false when
// none opens (the square has no forest and no corridor: a plain tile).
bool s3d_landcover_raster(const std::vector<std::string> &candidates, int px, float square_m, std::vector<uint8_t> &rgb);
