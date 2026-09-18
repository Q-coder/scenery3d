#ifndef S3D_WATER_MANAGER_H
#define S3D_WATER_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/ref.hpp>

#include "s3d_elevation_db.h"
#include "s3d_frame.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <limits>
#include <string>

namespace s3d
{

	// Manages water tiles (rivers + lakes): loads GLB files with per-vertex
	// colour in the background and parents them to the scene. Much simpler
	// than the building manager — a single LOD, no per-object controls, and
	// vertex colours are preserved via a shared StandardMaterial3D.
	class S3DWaterManager : public godot::Node3D
	{
		GDCLASS(S3DWaterManager, godot::Node3D);

	private:
		// Parsed water surface (one per tile).
		struct SurfaceData {
			godot::PackedVector3Array vertices;
			godot::PackedVector3Array normals;
			godot::PackedColorArray colors;
			godot::PackedInt32Array indices;
		};

		struct TileState {
			godot::MeshInstance3D *node = nullptr;
			godot::Ref<godot::ArrayMesh> mesh;
			SurfaceData baked;
			double dx = 0.0;
			double dz = 0.0;
			// Placement (see S3DFrame::place): vertices are flat offsets from
			// the manifest conversion origin (anchor); the tile is centred on
			// center_*; root_y is the local-up offset the root was placed with.
			double anchor_e = 0.0, anchor_n = 0.0;
			double center_e = 0.0, center_n = 0.0;
			double root_y = 0.0;
			bool drape_pending = false;
			bool loading = false;
			bool loaded = false;
			bool no_data = false;
			int drape_failures = 0;
			bool warned = false;
		};

		struct LoadRequest {
			std::string tile_id;
			std::string path;
			int distance;
		};

		struct LoadResult {
			std::string tile_id;
			bool success = false;
			SurfaceData surface;
		};

		struct ManifestEntry {
			std::string file;
			std::string tile_id;
			double center_e = 0;
			double center_n = 0;
			int polygons = 0;   // lake + river polygon count
			int streams = 0;
		};

		// One manifest per configured water directory.
		struct ManifestGroup {
			godot::String path;
			double conv_origin_e = 0.0;
			double conv_origin_n = 0.0;
			bool elevation_baked = false; // true → vertex Y already carries ASL + offset
			std::unordered_map<std::string, ManifestEntry> entries;
			// Axis-aligned bounding box (LV95 east / north) covering all
			// entries in this manifest. Used to suppress duplicate water
			// polygons in overlap zones (e.g. the Bodensee is modelled by
			// both the Swiss and Baden-Württemberg manifests).
			double bbox_min_e = 0.0, bbox_max_e = 0.0;
			double bbox_min_n = 0.0, bbox_max_n = 0.0;
			bool bbox_valid = false;
		};

		std::vector<ManifestGroup> manifests;
		bool manifest_loaded = false;

		// Tile state keyed by "<group_index>:<tile_id>".
		std::unordered_map<std::string, TileState> tiles;

		// Background worker.
		static constexpr int NUM_WORKERS = 2;
		std::vector<std::thread> worker_threads;
		std::mutex work_mutex;
		std::condition_variable work_cv;
		std::deque<LoadRequest> work_queue;
		std::mutex results_mutex;
		std::deque<LoadResult> results_queue;
		std::atomic<bool> worker_running{false};

		// Shared materials. `water_material` is the flat vertex-colour
		// fallback; `depth_material` shades the surface by how deep the
		// terrain behind it lies, which is what makes a bathymetry model
		// visible from the air.
		godot::Ref<godot::StandardMaterial3D> water_material;
		godot::Ref<godot::ShaderMaterial> depth_material;

		// Configuration.
		godot::String water_path;
		godot::PackedStringArray water_paths;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		S3DFrame frame;  // world frame (flat legacy / ECEF), origin_* mirror into it
		int load_radius_m = 8000;
		int far_radius_m = 50000;    // Lakes/river polygons visible at distance.
		int unload_margin_m = 2000;
		double vertical_offset_m = 1.5;
		// Same patch of ground modelled by two sources ⇒ keep the first only.
		bool dedup_overlapping_tiles = true;
		// Baked triangles taller than this are bake bugs and get discarded.
		double max_triangle_span_m = 50.0;
		bool depth_shading = true;
		godot::Color shallow_color = godot::Color(0.30f, 0.56f, 0.60f);
		godot::Color deep_color = godot::Color(0.02f, 0.09f, 0.17f);
		double opaque_depth_m = 16.0;
		double shore_alpha = 0.35;
		double deep_alpha = 0.97;

		// Optional elevation DB: when set, water vertices are draped onto
		// the terrain heightmap. Without it water sits flat at
		// vertical_offset_m which is only useful for already-baked GLBs.
		godot::Ref<S3DElevationDB> elevation_db;
		uint64_t elevation_epoch_seen = 0;

		// Flat-fallback Y for tiles whose DTM hasn't loaded after several
		// retries. NaN until first successful drape.
		double last_known_terrain_y = std::numeric_limits<double>::quiet_NaN();

		// Last known camera position in LV95.
		double last_cam_e = 0;
		double last_cam_n = 0;

		void start_worker();
		void stop_worker();
		void worker_func();

		void load_manifests();
		void update_tiles(godot::Vector3 camera_pos);
		void process_load_results();
		bool apply_drape(TileState &state, bool &out_fully_resolved);
		void retry_pending_drapes();

		void build_depth_material();
		void push_depth_uniforms();
		godot::Ref<godot::Material> surface_material() const;
		void reapply_material();

		static bool parse_glb(const std::vector<uint8_t> &data, SurfaceData &out);

	protected:
		static void _bind_methods();

	public:
		S3DWaterManager();
		~S3DWaterManager();

		void _process(double delta) override;

		void set_water_path(const godot::String &p_path);
		godot::String get_water_path() const;

		void set_water_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_water_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;
		// 0 = flat legacy frame, 1 = ECEF (see S3DFrame).
		void set_frame_mode(int p_mode);
		int get_frame_mode() const { return frame.mode; }

		void set_load_radius_m(int p_radius);
		int get_load_radius_m() const;

		void set_far_radius_m(int p_radius);
		int get_far_radius_m() const;

		void set_vertical_offset_m(double p_offset);
		double get_vertical_offset_m() const;

		void set_dedup_overlapping_tiles(bool p_enabled);
		bool get_dedup_overlapping_tiles() const;

		void set_max_triangle_span_m(double p_span);
		double get_max_triangle_span_m() const;

		void set_depth_shading(bool p_enabled);
		bool get_depth_shading() const;

		void set_shallow_color(const godot::Color &p_color);
		godot::Color get_shallow_color() const;

		void set_deep_color(const godot::Color &p_color);
		godot::Color get_deep_color() const;

		void set_opaque_depth_m(double p_depth);
		double get_opaque_depth_m() const;

		void set_shore_alpha(double p_alpha);
		double get_shore_alpha() const;

		void set_deep_alpha(double p_alpha);
		double get_deep_alpha() const;

		void set_elevation_db(const godot::Ref<S3DElevationDB> &p_db) { elevation_db = p_db; }

		int get_active_tile_count() const;
	};

}

#endif // S3D_WATER_MANAGER_H
