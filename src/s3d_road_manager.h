#ifndef S3D_ROAD_MANAGER_H
#define S3D_ROAD_MANAGER_H

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
#include <string>
#include <limits>

namespace s3d
{

	// Manages road tiles (roads and railways): loads GLB files with per-vertex
	// colour in the background and parents them to the scene. Much simpler
	// than the building manager — a single LOD, no per-object controls, and
	// vertex colours are preserved via a shared StandardMaterial3D.
	class S3DRoadManager : public godot::Node3D
	{
		GDCLASS(S3DRoadManager, godot::Node3D);

	private:
		// Parsed road surface (one per tile).
		struct SurfaceData {
			godot::PackedVector3Array vertices;
			godot::PackedVector3Array normals;
			godot::PackedColorArray colors;
			godot::PackedInt32Array indices;
		};

		struct TileState {
			godot::MeshInstance3D *node = nullptr;
			godot::Ref<godot::ArrayMesh> mesh;
			SurfaceData baked;     // pristine vertices for re-drape
			double dx = 0.0;       // cached LV95 origin offset
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
			int segments = 0;
		};

		// One manifest per configured road directory.
		struct ManifestGroup {
			godot::String path;
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
		godot::Ref<godot::StandardMaterial3D> road_material;

		// Configuration.
		godot::String road_path;
		godot::PackedStringArray road_paths;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		int load_radius_m = 12000;
		int unload_margin_m = 2000;
		double vertical_offset_m = 0.4;

		// Optional elevation DB: when set, road vertices are re-draped onto
		// the terrain heightmap at load time to remove DTM-mismatch gaps
		// between the baked road elevation and the rendered terrain.
		godot::Ref<S3DElevationDB> elevation_db;

		// Last seen elevation_db epoch; retry only fires when the DB has
		// new tiles to offer (avoids per-frame mesh rebuilds).
		uint64_t elevation_epoch_seen = 0;

		// Running mean of recently resolved DTM samples — used as a flat
		// fallback Y so road tiles in uncovered regions are still visible
		// (instead of staying invisible forever). NaN until first success.
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
		// Drape baked vertices onto current elevation_db. Returns true if
		// every vertex hit valid terrain (so the tile no longer needs retry).
		bool apply_drape(TileState &state, bool &out_fully_resolved);
		void retry_pending_drapes();

		static bool parse_glb(const std::vector<uint8_t> &data, SurfaceData &out);

	protected:
		static void _bind_methods();

	public:
		S3DRoadManager();
		~S3DRoadManager();

		void _process(double delta) override;

		void set_road_path(const godot::String &p_path);
		godot::String get_road_path() const;

		void set_road_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_road_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_load_radius_m(int p_radius);
		int get_load_radius_m() const;

		void set_vertical_offset_m(double p_offset);
		double get_vertical_offset_m() const;

		void set_elevation_db(const godot::Ref<S3DElevationDB> &p_db) { elevation_db = p_db; }

		int get_active_tile_count() const;
	};

}

#endif // S3D_ROAD_MANAGER_H
