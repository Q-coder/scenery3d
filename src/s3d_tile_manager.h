#ifndef S3D_TILE_MANAGER_H
#define S3D_TILE_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

#include "s3d_elevation_db.h"

namespace godot
{

	class S3DTile;

	// Manages the lifecycle of terrain tiles with distance-based LOD and
	// background file I/O. Tiles are organized in LOD rings: closer tiles
	// get higher detail, distant tiles get progressively coarser meshes.
	class S3DTileManager : public Node3D
	{
		GDCLASS(S3DTileManager, Node3D);

	private:
		// LOD ring: tiles within this Chebyshev distance get this LOD level.
		struct LODRing {
			int radius;
			int lod_level;
		};

		// Per-tile tracking state.
		struct TileState {
			S3DTile *node = nullptr;
			int current_lod = -1;
			int desired_lod = -1;
			bool loading = false;
			bool no_data = false; // File doesn't exist for this tile position.
		};

		// Background file load request.
		struct LoadRequest {
			int ei, ni;
			uint64_t key;
			std::string path;
			int tile_size;
			int desired_lod;
			int distance;
		};

		// Background file load result.
		struct LoadResult {
			int ei, ni;
			uint64_t key;
			std::vector<uint8_t> raw_bytes;
			int tile_size;
			int desired_lod;
			bool success = false;
		};

		// Active tile states keyed by tile_key(ei, ni).
		std::unordered_map<uint64_t, TileState> tiles;

		// LOD ring configuration (sorted inner to outer).
		std::vector<LODRing> lod_rings;

		// Background worker threads for file I/O.
		static constexpr int NUM_WORKERS = 4;
		std::vector<std::thread> worker_threads;
		std::mutex work_mutex;
		std::condition_variable work_cv;
		std::deque<LoadRequest> work_queue;
		std::mutex results_mutex;
		std::deque<LoadResult> results_queue;
		std::atomic<bool> worker_running{false};

		// Shared material for all terrain tiles.
		Ref<StandardMaterial3D> shared_material;

		// Configuration.
		int tile_size = 1024;
		int load_radius = 8;
		int load_budget = 4;
		int unload_margin = 2;
		String data_path;

		double origin_east = 2600000.0;
		double origin_north = 1200000.0;

		// Tiles at this LOD or coarser discard their heightmap after meshing.
		static constexpr int LOD_DISCARD_THRESHOLD = 4;

		// Max vertices to generate per frame (limits mesh build cost).
		static constexpr int VERTEX_BUDGET_PER_FRAME = 200000;

		Ref<S3DElevationDB> elevation_db;

		static inline uint64_t tile_key(int tx, int tz) {
			return ((uint64_t)(uint32_t)tx << 32) | (uint32_t)tz;
		}

		void rebuild_lod_rings();
		int lod_for_distance(int dist) const;

		void start_worker();
		void stop_worker();
		void worker_func();
		void process_load_results(int &verts_generated);

	protected:
		static void _bind_methods();

	public:
		S3DTileManager();
		~S3DTileManager();

		void _process(double delta) override;

		void update_tiles(Vector3 camera_pos);
		int get_active_tile_count() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_load_radius(int p_radius);
		int get_load_radius() const;

		void set_load_budget(int p_budget);
		int get_load_budget() const;

		void set_unload_margin(int p_margin);
		int get_unload_margin() const;

		void set_data_path(const String &p_path);
		String get_data_path() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		void set_elevation_db(Ref<S3DElevationDB> p_db);
		Ref<S3DElevationDB> get_elevation_db() const;
	};

}

#endif // S3D_TILE_MANAGER_H
