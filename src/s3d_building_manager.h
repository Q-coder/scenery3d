#ifndef S3D_BUILDING_MANAGER_H
#define S3D_BUILDING_MANAGER_H

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

	// Manages building tiles: loads GLB files in the background,
	// provides individual building visibility control, and handles
	// LOD switching between detailed meshes and simple boxes.
	class S3DBuildingManager : public Node3D
	{
		GDCLASS(S3DBuildingManager, Node3D);

	public:
		// Vertex/normal/index triple for one mesh surface (public for static helpers).
		struct SurfaceData {
			PackedVector3Array vertices;
			PackedVector3Array normals;
			PackedInt32Array indices;
		};

	private:
		// A single building within a tile.
		struct BuildingInfo {
			MeshInstance3D *node = nullptr;
			String uuid;
			bool user_hidden = false; // Hidden by user (replaced with bespoke model).
		};

		// State of a loaded building tile.
		struct TileState {
			Node3D *root_node = nullptr;          // Container for individual buildings.
			MeshInstance3D *far_lod_node = nullptr; // Merged box mesh for far rendering.
			std::vector<BuildingInfo> buildings;
			bool loading = false;
			bool loaded = false;
			bool no_data = false;
			bool has_detail = false; // Individual building meshes created.
			int lod = -1; // 0=detail, 1=boxes, 2=far-merged
		};

		// Background GLB load request.
		struct LoadRequest {
			std::string tile_id;
			std::string path;
			int distance;
		};

		// Background GLB load result.
		struct LoadResult {
			std::string tile_id;
			bool success = false;
			// Parsed building data (ready for Godot mesh creation on main thread).
			struct BuildingData {
				String uuid;
				// LOD0: detail mesh (wall + roof surfaces)
				SurfaceData lod0_wall;
				SurfaceData lod0_roof;
				// LOD1: box mesh (wall + roof surfaces)
				SurfaceData lod1_wall;
				SurfaceData lod1_roof;
			};
			std::vector<BuildingData> buildings;
			// Far-LOD merged mesh (wall + roof surfaces)
			SurfaceData far_wall;
			SurfaceData far_roof;
		};

		// Tile manifest entry.
		struct ManifestEntry {
			std::string file;
			std::string tile_id;
			double center_e = 0;
			double center_n = 0;
			int building_count = 0;
		};

		// Loaded manifest.
		std::unordered_map<std::string, ManifestEntry> manifest;
		bool manifest_loaded = false;
		// Origin used by the building tile vertices (read from manifest).
		// Tile root nodes are offset by (origin_east - manifest_origin_e,
		// 0, manifest_origin_n - origin_north) so they end up at the right
		// place in world coordinates regardless of the runtime origin.
		double manifest_origin_e = 2600000.0;
		double manifest_origin_n = 1200000.0;

		// Active tile states keyed by tile_id.
		std::unordered_map<std::string, TileState> tiles;

		// Set of UUIDs hidden by user.
		std::unordered_set<std::string> hidden_buildings;

		// Background worker.
		static constexpr int NUM_WORKERS = 2;
		std::vector<std::thread> worker_threads;
		std::mutex work_mutex;
		std::condition_variable work_cv;
		std::deque<LoadRequest> work_queue;
		std::mutex results_mutex;
		std::deque<LoadResult> results_queue;
		std::atomic<bool> worker_running{false};

		// Shared materials.
		Ref<StandardMaterial3D> wall_material;
		Ref<StandardMaterial3D> roof_material;

		// Last known camera position in LV95 (for process_load_results).
		double last_cam_e = 0;
		double last_cam_n = 0;

		// Configuration.
		String buildings_path;
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		int load_radius_m = 8000;    // Distance in meters to load detailed buildings.
		int far_radius_m = 30000;    // Distance for far-LOD boxes.
		int unload_margin_m = 2000;  // Hysteresis margin.

		// LOD thresholds (meters from camera).
		int detail_radius_m = 3000;  // Within this: LOD0 (full detail).
		// Between detail_radius and load_radius: LOD1 (individual boxes).
		// Between load_radius and far_radius: LOD2 (far merged mesh).

		void start_worker();
		void stop_worker();
		void worker_func();

		void load_manifest();
		void update_buildings(Vector3 camera_pos);
		void process_load_results();

		// Parse a GLB binary buffer into a LoadResult (called on worker thread).
		static LoadResult parse_glb(const std::string &tile_id,
		                            const std::vector<uint8_t> &data);

	protected:
		static void _bind_methods();

	public:
		S3DBuildingManager();
		~S3DBuildingManager();

		void _process(double delta) override;

		// Hide/show individual buildings by UUID.
		void hide_building(const String &uuid);
		void show_building(const String &uuid);
		bool is_building_hidden(const String &uuid) const;
		TypedArray<String> get_hidden_buildings() const;

		// Getters/Setters.
		void set_buildings_path(const String &p_path);
		String get_buildings_path() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_load_radius_m(int p_radius);
		int get_load_radius_m() const;

		void set_far_radius_m(int p_radius);
		int get_far_radius_m() const;

		void set_detail_radius_m(int p_radius);
		int get_detail_radius_m() const;

		int get_active_tile_count() const;
		int get_building_count() const;
	};

}

#endif // S3D_BUILDING_MANAGER_H
