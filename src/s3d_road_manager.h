#ifndef S3D_ROAD_MANAGER_H
#define S3D_ROAD_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/ref.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

namespace godot
{

	// Manages road tiles (roads and railways): loads GLB files with per-vertex
	// colour in the background and parents them to the scene. Much simpler
	// than the building manager — a single LOD, no per-object controls, and
	// vertex colours are preserved via a shared StandardMaterial3D.
	class S3DRoadManager : public Node3D
	{
		GDCLASS(S3DRoadManager, Node3D);

	private:
		// Parsed road surface (one per tile).
		struct SurfaceData {
			PackedVector3Array vertices;
			PackedVector3Array normals;
			PackedColorArray colors;
			PackedInt32Array indices;
		};

		struct TileState {
			MeshInstance3D *node = nullptr;
			bool loading = false;
			bool loaded = false;
			bool no_data = false;
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
		Ref<StandardMaterial3D> road_material;

		// Configuration.
		String road_path;
		PackedStringArray road_paths;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		int load_radius_m = 12000;
		int unload_margin_m = 2000;
		double vertical_offset_m = 2.0;

		// Last known camera position in LV95.
		double last_cam_e = 0;
		double last_cam_n = 0;

		void start_worker();
		void stop_worker();
		void worker_func();

		void load_manifests();
		void update_tiles(Vector3 camera_pos);
		void process_load_results();

		static bool parse_glb(const std::vector<uint8_t> &data, SurfaceData &out);

	protected:
		static void _bind_methods();

	public:
		S3DRoadManager();
		~S3DRoadManager();

		void _process(double delta) override;

		void set_road_path(const String &p_path);
		String get_road_path() const;

		void set_road_paths(const PackedStringArray &p_paths);
		PackedStringArray get_road_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_load_radius_m(int p_radius);
		int get_load_radius_m() const;

		void set_vertical_offset_m(double p_offset);
		double get_vertical_offset_m() const;

		int get_active_tile_count() const;
	};

}

#endif // S3D_ROAD_MANAGER_H
