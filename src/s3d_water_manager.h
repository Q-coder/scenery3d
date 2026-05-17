#ifndef S3D_WATER_MANAGER_H
#define S3D_WATER_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/ref.hpp>

#include "s3d_elevation_db.h"

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

namespace godot
{

	// Manages water tiles (rivers + lakes): loads GLB files with per-vertex
	// colour in the background and parents them to the scene. Much simpler
	// than the building manager — a single LOD, no per-object controls, and
	// vertex colours are preserved via a shared StandardMaterial3D.
	class S3DWaterManager : public Node3D
	{
		GDCLASS(S3DWaterManager, Node3D);

	private:
		// Parsed water surface (one per tile).
		struct SurfaceData {
			PackedVector3Array vertices;
			PackedVector3Array normals;
			PackedColorArray colors;
			PackedInt32Array indices;
		};

		struct TileState {
			MeshInstance3D *node = nullptr;
			Ref<ArrayMesh> mesh;
			SurfaceData baked;
			double dx = 0.0;
			double dz = 0.0;
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
			String path;
			double conv_origin_e = 0.0;
			double conv_origin_n = 0.0;
			std::unordered_map<std::string, ManifestEntry> entries;
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

		// Shared material.
		Ref<StandardMaterial3D> water_material;

		// Configuration.
		String water_path;
		PackedStringArray water_paths;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		int load_radius_m = 12000;
		int far_radius_m = 50000;    // Lakes/river polygons visible at distance.
		int unload_margin_m = 2000;
		double vertical_offset_m = 1.5;

		// Optional elevation DB: when set, water vertices are draped onto
		// the terrain heightmap. Without it water sits flat at
		// vertical_offset_m which is only useful for already-baked GLBs.
		Ref<S3DElevationDB> elevation_db;
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
		void update_tiles(Vector3 camera_pos);
		void process_load_results();
		bool apply_drape(TileState &state, bool &out_fully_resolved);
		void retry_pending_drapes();

		static bool parse_glb(const std::vector<uint8_t> &data, SurfaceData &out);

	protected:
		static void _bind_methods();

	public:
		S3DWaterManager();
		~S3DWaterManager();

		void _process(double delta) override;

		void set_water_path(const String &p_path);
		String get_water_path() const;

		void set_water_paths(const PackedStringArray &p_paths);
		PackedStringArray get_water_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_load_radius_m(int p_radius);
		int get_load_radius_m() const;

		void set_far_radius_m(int p_radius);
		int get_far_radius_m() const;

		void set_vertical_offset_m(double p_offset);
		double get_vertical_offset_m() const;

		void set_elevation_db(const Ref<S3DElevationDB> &p_db) { elevation_db = p_db; }

		int get_active_tile_count() const;
	};

}

#endif // S3D_WATER_MANAGER_H
