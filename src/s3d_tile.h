#ifndef S3D_TILE_H
#define S3D_TILE_H

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include "s3d_frame.h"
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

namespace s3d
{

	// Represents a single terrain tile as a mesh generated from heightmap data.
	// The heightmap image should be FORMAT_RF (32-bit float per pixel) with
	// values representing elevation in meters. The mesh spans [0, tile_size]
	// in X and Z, with Y taken directly from pixel values.
	class S3DTile : public godot::MeshInstance3D
	{
		GDCLASS(S3DTile, godot::MeshInstance3D);

	private:
		int tile_x = 0;
		int tile_z = 0;
		int tile_size = 0;
		int lod_level = 4;
		bool mesh_ready = false;

		godot::Ref<godot::Image> heightmap;
		godot::Ref<godot::Material> material;

		// Placement (see S3DFrame): the mesh is authored as flat offsets from
		// grid anchor (anchor_e, anchor_n) [local x = anchor_e - E, z = N -
		// anchor_n], centred on (center_e, center_n). In ECEF mode the node
		// transform carries the WUN frame + grid Jacobian and generate_mesh()
		// subtracts the curvature sag; with `exact` (used for 8 km chunks)
		// every vertex is instead projected individually and expressed in
		// the WUN frame at the centre, so a chunk has no affine seam error.
		S3DFrame frame;
		double anchor_e = 0.0, anchor_n = 0.0;
		double center_e = 0.0, center_n = 0.0;
		double y_offset = 0.0;
		bool exact = false;
		bool placed = false;
		godot::Transform3D exact_wun;  // WUN frame at the centre (ECEF modes)
		// ECEF non-exact ("corner") mode: the four tile corners are projected
		// exactly and expressed in exact_wun; interior vertices interpolate
		// them bilinearly, so a shared edge between neighbours is computed
		// from the same two corner points on both sides (no seam), while the
		// per-vertex cost stays a handful of multiplies. corner[0]=(x0,z0),
		// [1]=(x1,z0), [2]=(x0,z1), [3]=(x1,z1) in mesh-local terms.
		godot::Vector3 corner[4];

		// Sample height at pixel coordinates, clamped to image bounds.
		godot::real_t sample_height(int px, int py, int img_w, int img_h) const;

	protected:
		static void _bind_methods();

	public:
		S3DTile();
		~S3DTile();

		// Build the terrain mesh from the stored heightmap.
		void generate_mesh();

		// Place the tile (sets the node transform; see the placement fields).
		// Must be called before generate_mesh() in ECEF mode.
		void set_placement(const S3DFrame &p_frame, double p_anchor_e, double p_anchor_n,
			double p_center_e, double p_center_n, double p_y_offset, bool p_exact);

		// Store the heightmap image (does NOT auto-generate mesh).
		void set_heightmap(godot::Ref<godot::Image> p_heightmap);
		godot::Ref<godot::Image> get_heightmap() const;

		// Set shared material (used by generate_mesh if set).
		void set_material(godot::Ref<godot::Material> p_mat);
		godot::Ref<godot::Material> get_material() const;

		// Returns true once the mesh has been generated.
		bool is_mesh_ready() const;

		void set_lod_level(int p_lod);
		int get_lod_level() const;
		// LOD levels name a target vertex spacing of 2^lod metres (the ring
		// table was written for 1 m/px tiles). A tile stored at a coarser
		// pixel resolution reaches that spacing with a smaller stride:
		// a 34 px / 1024 m tile (31 m/px) stays at stride 1 for every ring
		// up to LOD 4 (16 m target) and uses stride 2 at LOD 6 (64 m).
		// Before this, ring LODs were applied as raw strides to every tile,
		// so the 30 m base packs rendered 256 m–1 km cells inside 16 km
		// while the far chunks beyond used 250 m — the Alps morphed at
		// every ring (2026-09-20).
		static int effective_lod(int lod, int img_w, int tile_size_m);

		void set_tile_x(int p_x);
		int get_tile_x() const;

		void set_tile_z(int p_z);
		int get_tile_z() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;
	};

}

#endif // S3D_TILE_H
