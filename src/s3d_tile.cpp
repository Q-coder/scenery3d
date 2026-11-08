#include "s3d_tile.h"
#include <cmath>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

namespace s3d {

S3DTile::S3DTile()
{
}

S3DTile::~S3DTile()
{
}

void S3DTile::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("generate_mesh"), &S3DTile::generate_mesh);

	ClassDB::bind_method(D_METHOD("set_heightmap", "heightmap"), &S3DTile::set_heightmap);
	ClassDB::bind_method(D_METHOD("get_heightmap"), &S3DTile::get_heightmap);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "heightmap", PROPERTY_HINT_RESOURCE_TYPE, "Image"), "set_heightmap", "get_heightmap");

	ClassDB::bind_method(D_METHOD("is_mesh_ready"), &S3DTile::is_mesh_ready);

	ClassDB::bind_method(D_METHOD("set_lod_level", "lod"), &S3DTile::set_lod_level);
	ClassDB::bind_method(D_METHOD("get_lod_level"), &S3DTile::get_lod_level);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "lod_level"), "set_lod_level", "get_lod_level");

	ClassDB::bind_method(D_METHOD("set_tile_x", "x"), &S3DTile::set_tile_x);
	ClassDB::bind_method(D_METHOD("get_tile_x"), &S3DTile::get_tile_x);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_x"), "set_tile_x", "get_tile_x");

	ClassDB::bind_method(D_METHOD("set_tile_z", "z"), &S3DTile::set_tile_z);
	ClassDB::bind_method(D_METHOD("get_tile_z"), &S3DTile::get_tile_z);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_z"), "set_tile_z", "get_tile_z");

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DTile::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DTile::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");
}

real_t S3DTile::sample_height(int px, int py, int img_w, int img_h) const
{
	px = px < 0 ? 0 : (px >= img_w ? img_w - 1 : px);
	py = py < 0 ? 0 : (py >= img_h ? img_h - 1 : py);
	return static_cast<real_t>(heightmap->get_pixel(px, py).r);
}

void S3DTile::generate_mesh()
{
	if (heightmap.is_null() || tile_size <= 0) {
		return;
	}

	int img_w = heightmap->get_width();
	int img_h = heightmap->get_height();

	if (img_w < 2 || img_h < 2) {
		return;
	}

	// Placement-dependent vertex mapping (see set_placement).
	const bool curved_exact = placed && frame.ecef() && exact;
	const bool curved_corner = placed && frame.ecef() && !exact;
	const Basis exact_inv = exact_wun.basis.transposed();
	const Basis exact_grid_basis = (curved_exact || curved_corner) ? frame.grid_basis_at(center_e, center_n) : Basis();
	const double ts_d = (double)tile_size;

	// Compute stride from LOD level. Higher LOD = coarser mesh. Clamp so a
	// low-resolution tile still yields at least a 2x2 mesh: a 34 px Norway
	// tile at LOD 6 (stride 64) otherwise produced NO mesh at all — an
	// invisible tile the far chunk had already been dropped for, i.e. a hole
	// (seen as sky through the suppressed globe south of Oslo, 2026-09-18).
	int stride = 1 << effective_lod(lod_level, img_w, tile_size);
	if (stride > img_w - 1) stride = img_w - 1;
	if (stride > img_h - 1) stride = img_h - 1;
	if (stride < 1) stride = 1;

	// Number of vertices along each axis after applying stride.
	int verts_x = (img_w - 1) / stride + 1;
	int verts_z = (img_h - 1) / stride + 1;

	if (verts_x < 2 || verts_z < 2) {
		return;
	}

	int vertex_count = verts_x * verts_z;
	int quad_count = (verts_x - 1) * (verts_z - 1);
	int index_count = quad_count * 6;

	PackedVector3Array vertices;
	PackedVector3Array normals;
	PackedVector2Array uvs;
	PackedInt32Array indices;

	vertices.resize(vertex_count);
	normals.resize(vertex_count);
	uvs.resize(vertex_count);
	indices.resize(index_count);

	real_t ts = static_cast<real_t>(tile_size);

	// Spacing between vertices in world units.
	real_t spacing_x = ts / static_cast<real_t>(verts_x - 1);
	real_t spacing_z = ts / static_cast<real_t>(verts_z - 1);

	// Spacing in pixel units (used for normal computation via central differences).
	real_t pixel_spacing_x = ts / static_cast<real_t>(img_w - 1);
	real_t pixel_spacing_z = ts / static_cast<real_t>(img_h - 1);

	// Fast raw access to the FORMAT_RF heightmap (one float per pixel). This is
	// ~50× faster than Image::get_pixel() and is what makes the per-vertex
	// block-average below cheap enough to run on the main thread.
	PackedByteArray raw_data = heightmap->get_data();
	const float *hf = nullptr;
	if (heightmap->get_format() == Image::FORMAT_RF &&
		(int64_t)raw_data.size() >= (int64_t)img_w * img_h * 4) {
		hf = reinterpret_cast<const float *>(raw_data.ptr());
	}

	// Nearest-pixel sample (clamped) straight from the raw float buffer.
	auto sample_raw = [&](int x, int y) -> real_t {
		if (!hf) return sample_height(x, y, img_w, img_h);
		x = x < 0 ? 0 : (x >= img_w ? img_w - 1 : x);
		y = y < 0 ? 0 : (y >= img_h ? img_h - 1 : y);
		return static_cast<real_t>(hf[(size_t)y * img_w + x]);
	};

	// Block-averaged height for a coarse-LOD vertex. Averaging a stride-sized
	// window (instead of picking a single pixel) makes the coarse mesh track
	// the true full-resolution surface far more closely, so baked roads/water
	// (which sit at full-res elevation) stop floating and the aircraft (placed
	// on the full-res elevation DB) stops sinking into humps at mid distance.
	// CRITICAL: vertices on the tile BORDER must stay nearest-pixel so they
	// match the identical border samples of the neighbouring tile — otherwise
	// averaging would pull each side toward its own interior and tear a visible
	// crack along every tile seam. Interior vertices average a centred window.
	int half = stride / 2;
	auto sample_height_lod = [&](int px, int py) -> real_t {
		if (half <= 0 || px <= 0 || py <= 0 || px >= img_w - 1 || py >= img_h - 1) {
			return sample_raw(px, py); // border or LOD0 → exact, crack-safe
		}
		int x0 = px - half; if (x0 < 0) x0 = 0;
		int x1 = px + half; if (x1 > img_w - 1) x1 = img_w - 1;
		int y0 = py - half; if (y0 < 0) y0 = 0;
		int y1 = py + half; if (y1 > img_h - 1) y1 = img_h - 1;
		double sum = 0.0;
		int n = 0;
		for (int y = y0; y <= y1; y++) {
			for (int x = x0; x <= x1; x++) {
				sum += hf[(size_t)y * img_w + x];
				n++;
			}
		}
		return static_cast<real_t>(sum / (double)n);
	};

	// Generate vertices, normals, and UVs.
	for (int gz = 0; gz < verts_z; gz++) {
		// Map gz linearly to pixel row: 0 -> 0, verts_z-1 -> img_h-1.
		int py = gz * (img_h - 1) / (verts_z - 1);

		for (int gx = 0; gx < verts_x; gx++) {
			// Map gx linearly to pixel col: 0 -> 0, verts_x-1 -> img_w-1.
			int px = gx * (img_w - 1) / (verts_x - 1);

			int idx = gz * verts_x + gx;

			real_t world_x = static_cast<real_t>(gx) * spacing_x;
			real_t world_z = static_cast<real_t>(gz) * spacing_z;
			real_t height = sample_height_lod(px, py);

			if (curved_exact) {
				// Project this grid point onto the ellipsoid and express it
				// in the WUN frame at the tile centre (node transform).
				const double e = anchor_e - (double)world_x;
				const double n = anchor_n + (double)world_z;
				const Vector3 w = frame.grid_to_world(e, n, (double)height);
				vertices[idx] = exact_inv.xform(w - exact_wun.origin) - Vector3(0.0, y_offset, 0.0);
			} else if (curved_corner) {
				// Bilinear blend of the exactly projected corners (WUN-local,
				// centred on the tile), minus the curvature sag.
				const double u = (double)world_x / ts_d, v = (double)world_z / ts_d;
				const double lx = (1.0 - u) * (1.0 - v) * corner[0].x + u * (1.0 - v) * corner[1].x + (1.0 - u) * v * corner[2].x + u * v * corner[3].x;
				const double lz = (1.0 - u) * (1.0 - v) * corner[0].z + u * (1.0 - v) * corner[1].z + (1.0 - u) * v * corner[2].z + u * v * corner[3].z;
				// Height goes up the LOCAL normal, which leans outward by
				// (lx, lz)/R relative to the centre's up: without this a
				// shared edge vertex 600 m ASL lands 5 cm apart on the two
				// neighbouring tiles (thin gap lines seen at Oslo).
				const double tilt = 1.0 + (double)height / S3DFrame::R_MEAN;
				vertices[idx] = Vector3(lx * tilt, (double)height - frame.sag(lx, lz), lz * tilt);
			} else {
				vertices[idx] = Vector3(world_x, height, world_z);
			}

			// UV: [0,1] range across the tile.
			real_t u = static_cast<real_t>(gx) / static_cast<real_t>(verts_x - 1);
			real_t v = static_cast<real_t>(gz) / static_cast<real_t>(verts_z - 1);
			uvs[idx] = Vector2(u, v);

			// Compute smooth normal using central differences on the heightmap.
			// Always sample at 1-pixel distance for high-quality normals,
			// independent of mesh LOD — this keeps crisp surface shading even
			// where the geometry itself is averaged/coarse.
			real_t h_left = sample_raw(px - 1, py);
			real_t h_right = sample_raw(px + 1, py);
			real_t h_down = sample_raw(px, py - 1);
			real_t h_up = sample_raw(px, py + 1);

			// The distance in world units between the two samples (2 pixels).
			real_t dx = pixel_spacing_x * 2.0;
			real_t dz = pixel_spacing_z * 2.0;

			Vector3 normal = Vector3(
				(h_left - h_right) / dx,
				1.0,
				(h_down - h_up) / dz
			);
			if (curved_exact || curved_corner) {
				// Grid-space normal -> WUN-local (convergence/scale of the grid).
				normal = exact_grid_basis.xform(normal);
			}
			normals[idx] = normal.normalized();
		}
	}

	// Generate triangle indices. Two triangles per quad, wound counter-clockwise
	// so normals face upward (matching Godot's front-face convention).
	int ii = 0;
	for (int gz = 0; gz < verts_z - 1; gz++) {
		for (int gx = 0; gx < verts_x - 1; gx++) {
			int top_left = gz * verts_x + gx;
			int top_right = top_left + 1;
			int bottom_left = (gz + 1) * verts_x + gx;
			int bottom_right = bottom_left + 1;

			// Godot's front face is CLOCKWISE. Seen from above (+Y) these two
			// triangles must wind clockwise, or the double-sided material treats
			// the terrain as back faces and flips its normals: the sun then never
			// lights it (only the ambient floor did, which is why the patch went
			// black from space where the ambient is dimmed — 2026-09-18).
			// First triangle (top-left, top-right, bottom-left).
			indices[ii++] = top_left;
			indices[ii++] = top_right;
			indices[ii++] = bottom_left;

			// Second triangle (top-right, bottom-right, bottom-left).
			indices[ii++] = top_right;
			indices[ii++] = bottom_right;
			indices[ii++] = bottom_left;
		}
	}

	// Build the ArrayMesh.
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = vertices;
	arrays[Mesh::ARRAY_NORMAL] = normals;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_INDEX] = indices;

	Ref<ArrayMesh> array_mesh;
	array_mesh.instantiate();
	array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

	// Skirts (surface 1): a vertical strip hanging from each border edge so
	// that T-junction cracks between neighbouring tiles of different LOD
	// (the finer edge's intermediate vertices lie above/below the coarser
	// edge's straight segment) and any residual seam never show the sky or
	// the globe through the terrain. Depth scales with the vertex spacing,
	// which bounds the crack height; both windings are emitted so the wall is
	// visible from either side regardless of the material's cull mode.
	{
		const real_t skirt_depth = MAX((real_t)20.0, (real_t)0.3 * MAX(spacing_x, spacing_z));
		PackedVector3Array sv;
		PackedVector3Array sn;
		PackedVector2Array suv;
		PackedInt32Array si;
		auto add_edge = [&](auto index_of, int count) {
			const int base = (int)sv.size();
			for (int k = 0; k < count; k++) {
				const int src = index_of(k);
				sv.push_back(vertices[src]);
				sn.push_back(normals[src]);
				suv.push_back(uvs[src]);
			}
			for (int k = 0; k < count; k++) {
				const int src = index_of(k);
				Vector3 d = vertices[src];
				d.y -= skirt_depth;
				sv.push_back(d);
				sn.push_back(normals[src]);
				suv.push_back(uvs[src]);
			}
			for (int k = 0; k + 1 < count; k++) {
				const int a = base + k, b = base + k + 1;
				const int c = base + count + k, d = base + count + k + 1;
				const int quad[12] = {a, c, b, b, c, d, a, b, c, b, d, c};
				for (int q = 0; q < 12; q++) si.push_back(quad[q]);
			}
		};
		add_edge([&](int k) { return k; }, verts_x);                                   // gz = 0 edge
		add_edge([&](int k) { return (verts_z - 1) * verts_x + k; }, verts_x);        // gz = max edge
		add_edge([&](int k) { return k * verts_x; }, verts_z);                        // gx = 0 edge
		add_edge([&](int k) { return k * verts_x + (verts_x - 1); }, verts_z);        // gx = max edge
		Array sarrays;
		sarrays.resize(Mesh::ARRAY_MAX);
		sarrays[Mesh::ARRAY_VERTEX] = sv;
		sarrays[Mesh::ARRAY_NORMAL] = sn;
		sarrays[Mesh::ARRAY_TEX_UV] = suv;
		sarrays[Mesh::ARRAY_INDEX] = si;
		array_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, sarrays);
	}

	// Apply material (both surfaces).
	if (material.is_valid()) {
		array_mesh->surface_set_material(0, material);
		array_mesh->surface_set_material(1, material);
	} else {
		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_albedo(Color(0.45, 0.55, 0.35));
		mat->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
		array_mesh->surface_set_material(0, mat);
		array_mesh->surface_set_material(1, mat);
	}

	set_mesh(array_mesh);
	mesh_ready = true;
}

void S3DTile::set_placement(const S3DFrame &p_frame, double p_anchor_e, double p_anchor_n,
	double p_center_e, double p_center_n, double p_y_offset, bool p_exact)
{
	frame = p_frame;
	anchor_e = p_anchor_e;
	anchor_n = p_anchor_n;
	center_e = p_center_e;
	center_n = p_center_n;
	y_offset = p_y_offset;
	exact = p_exact && frame.ecef();
	placed = true;
	if (frame.ecef()) {
		exact_wun = frame.wun_at(center_e, center_n);
		set_transform(exact_wun * Transform3D(Basis(), Vector3(0.0, y_offset, 0.0)));
		if (!exact) {
			// Corner mode: project the four mesh corners (mesh-local x,z in
			// {0, tile_size}) onto the ellipsoid and express them in the
			// centre's WUN frame.
			const Basis inv = exact_wun.basis.transposed();
			const double ts = (double)tile_size;
			const double xs[2] = {0.0, ts};
			const double zs[2] = {0.0, ts};
			for (int j = 0; j < 2; ++j) {
				for (int i = 0; i < 2; ++i) {
					const double e = anchor_e - xs[i];
					const double n = anchor_n + zs[j];
					Vector3 l = inv.xform(frame.grid_to_world(e, n, 0.0) - exact_wun.origin);
					l.y = 0.0;
					corner[j * 2 + i] = l;
				}
			}
		}
	} else {
		set_transform(frame.place(anchor_e, anchor_n, center_e, center_n, y_offset));
	}
}

void S3DTile::set_heightmap(Ref<Image> p_heightmap)
{
	heightmap = p_heightmap;
	mesh_ready = false;
}

void S3DTile::set_material(Ref<Material> p_mat)
{
	material = p_mat;
}

Ref<Material> S3DTile::get_material() const
{
	return material;
}

Ref<Image> S3DTile::get_heightmap() const
{
	return heightmap;
}

bool S3DTile::is_mesh_ready() const
{
	return mesh_ready;
}

int S3DTile::effective_lod(int lod, int img_w, int tile_size_m)
{
	if (lod < 0) lod = 0;
	if (img_w < 2 || tile_size_m <= 0) return lod;
	const double px_res = (double)tile_size_m / (double)(img_w - 1);
	const int res_lod = (int)std::lround(std::log2(std::max(px_res, 1.0)));
	const int e = lod - res_lod;
	return e < 0 ? 0 : e;
}

void S3DTile::set_lod_level(int p_lod)
{
	lod_level = p_lod < 0 ? 0 : p_lod;
}

int S3DTile::get_lod_level() const
{
	return lod_level;
}

void S3DTile::set_tile_x(int p_x)
{
	tile_x = p_x;
}

int S3DTile::get_tile_x() const
{
	return tile_x;
}

void S3DTile::set_tile_z(int p_z)
{
	tile_z = p_z;
}

int S3DTile::get_tile_z() const
{
	return tile_z;
}

void S3DTile::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DTile::get_tile_size() const
{
	return tile_size;
}

} // namespace s3d
