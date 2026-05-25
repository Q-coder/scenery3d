#ifndef S3D_TILE_MANAGER_H
#define S3D_TILE_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>

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

namespace s3d
{

	class S3DTile;

	// Manages the lifecycle of terrain tiles with distance-based LOD and
	// background file I/O. Tiles are organized in LOD rings: closer tiles
	// get higher detail, distant tiles get progressively coarser meshes.
	class S3DTileManager : public godot::Node3D
	{
		GDCLASS(S3DTileManager, godot::Node3D);

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
			// Applied ortho texture size in pixels (1024 / 256 / 64), or -1
			// if no ortho texture is on the tile yet. Used to decide when an
			// LOD change requires re-fetching the orthophoto from disk so
			// the resolution actually matches the new viewing distance.
			int current_ortho_pixels = -1;
			bool loading = false;
			bool no_data = false; // File doesn't exist for this tile position.
		};

		// Background file load request.
		struct LoadRequest {
			int ei, ni;
			uint64_t key;
			// Candidate file paths tried in order (first existing/openable wins).
			std::vector<std::string> paths;
			// Meters spanned by one tile along each LV95 axis (file naming uses this).
			int tile_size;
			// Pixels per axis stored in the raw heightmap file. Decoupled from
			// tile_size so a tile can cover N meters at M pixels/meter resolution.
			int tile_px;
			int desired_lod;
			int distance;

			// Orthophoto JPEG candidate paths (worker tries in order, first
			// openable wins). Empty = no ortho for this tile.
			std::vector<std::string> ortho_paths;
			// Target in-memory ortho texture size in pixels (-1 if no ortho).
			int ortho_pixels = -1;

			// Chunk fields (is_chunk == true).
			bool is_chunk = false;
			int chunk_grid_ei = 0;
			int chunk_grid_ni = 0;
			int chunk_tile_count = 0;
			int composite_res = 0;
			// Candidate base directories for chunk tile lookup.
			std::vector<std::string> base_paths;
			// Orthophoto mip directories for chunk compositing
			// (one per orthophoto root; first existing JPEG per sub-tile wins).
			std::vector<std::string> ortho_mip_dirs;
		};

		// Background file load result.
		struct LoadResult {
			int ei, ni;
			uint64_t key;
			std::vector<uint8_t> raw_bytes;
			// Mesh span in meters (regular: req.tile_size; chunk: csize * req.tile_size).
			int tile_size;
			// Heightmap image dimension in pixels (regular tile path only — chunks
			// build a composite at composite_res pixels independently).
			int tile_px = 0;
			int desired_lod;
			bool success = false;
			bool is_chunk = false;
			int composite_res = 0;
			// Orthophoto JPEG bytes (empty = no ortho).
			std::vector<uint8_t> jpeg_bytes;
			// Pre-decoded composited RGB bytes (used when skip_white_pixels
			// merges multiple sources in the worker). When non-empty,
			// jpeg_bytes is ignored. Layout: row-major RGB8, ortho_rgb_w² px.
			std::vector<uint8_t> ortho_rgb;
			int ortho_rgb_w = 0;
			// Target in-memory ortho texture size in pixels (-1 if none).
			int ortho_pixels = -1;
			// Per-tile ortho JPEGs for chunk compositing.
			// Key: (local_x * chunk_count + local_y), value: JPEG bytes.
			std::unordered_map<int, std::vector<uint8_t>> chunk_ortho_jpegs;
			int chunk_tile_count = 0;
		};

		// Active tile states keyed by tile_key(ei, ni).
		std::unordered_map<uint64_t, TileState> tiles;

		// Far terrain chunks keyed by tile_key(chunk_grid_ei, chunk_grid_ni).
		std::unordered_map<uint64_t, TileState> chunks;

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

		// Shared material for tiles without orthophoto.
		godot::Ref<godot::StandardMaterial3D> shared_material;

		// Ordered list of orthophoto root directories (each contains
		// ortho_E_N.jpg + mip subdirs). Worker tries each in order; the
		// first existing file wins. Lets CH SWISSIMAGE and DE LGL DOP20
		// coexist seamlessly across the border.
		godot::PackedStringArray orthophoto_paths;

		// When true and 2+ orthophoto paths are configured, the worker
		// decodes every candidate JPEG and composites them: pure white
		// (255,255,255) pixels in higher-priority sources are filled in
		// from lower-priority ones. SWISSIMAGE encodes German territory at
		// the CH/DE border as white pixels (up to 43% of a 1024² tile),
		// so this lets BW DOP20 fill those gaps cleanly.
		bool skip_white_pixels = true;

		// Build the list of candidate ortho file paths (one per data root)
		// for a given tile + LOD. Worker tries them in order.
		std::vector<std::string> ortho_paths_for_tile(int ei, int ni, int lod) const;

		// Configuration.
		// Meters per tile axis (drives LV95 tile-index math and file naming).
		int tile_size = 1024;
		// Pixels per tile axis in the raw heightmap files. 0 means "same as
		// tile_size" (legacy 1 m/px). Set explicitly when tiles store the
		// terrain at a different pixel resolution (e.g. 2 m/px → tile_px=512
		// for tile_size=1024).
		int tile_px = 0;
		int load_radius = 8;
		int far_radius = 200;
		int chunk_size = 8;
		int load_budget = 4;
		int unload_margin = 2;
		// Ordered list of directories to search for tile files.
		// The first entry is returned by get_data_path() for backward compat.
		godot::PackedStringArray data_paths;

		double origin_east = 2600000.0;
		double origin_north = 1200000.0;

		// Tiles at this LOD or coarser discard their heightmap after meshing.
		static constexpr int LOD_DISCARD_THRESHOLD = 4;

		// Max vertices to generate per frame (limits mesh build cost).
		static constexpr int VERTEX_BUDGET_PER_FRAME = 200000;

		// Max number of tile results that include a JPEG ortho (decode +
		// texture upload on the main thread) processed per frame. Without
		// this, crossing a chunk boundary lets dozens of decodes pile into
		// one frame and visibly stalls the simulation. Metal is especially
		// sensitive to upload bursts here, so keep this conservative.
		static constexpr int ORTHO_DECODES_PER_FRAME = 1;

		// Max number of chunk results processed per frame. Each chunk
		// decodes up to chunk_size² mip3 JPEGs and builds a composite
		// texture, so even one is moderately expensive.
		static constexpr int CHUNKS_PER_FRAME = 1;

		// Resolution of chunk composite heightmaps.
		static constexpr int CHUNK_COMPOSITE_RES = 33;

		// Y offset (metres) applied to chunk meshes so they sit just below
		// the individual-tile surface and lose the depth test wherever the
		// two overlap. Large enough to cover the error from 250 m chunk
		// sampling vs 1 m tile sampling on typical terrain.
		static constexpr double CHUNK_Y_BIAS = -30.0;

		godot::Ref<S3DElevationDB> elevation_db;

		static inline uint64_t tile_key(int tx, int tz) {
			return ((uint64_t)(uint32_t)tx << 32) | (uint32_t)tz;
		}

		void rebuild_lod_rings();
		int lod_for_distance(int dist) const;
		// Which ortho mip level (0/2/3) should be used for a tile at this LOD.
		static int ortho_mip_for_lod(int lod);

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

		void update_tiles(godot::Vector3 camera_pos);
		int get_active_tile_count() const;

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_tile_px(int p_px);
		int get_tile_px() const;
		// Effective pixel count per tile (tile_px if >0, else tile_size).
		int effective_tile_px() const { return tile_px > 0 ? tile_px : tile_size; }

		void set_load_radius(int p_radius);
		int get_load_radius() const;

		void set_far_radius(int p_radius);
		int get_far_radius() const;

		void set_load_budget(int p_budget);
		int get_load_budget() const;

		void set_unload_margin(int p_margin);
		int get_unload_margin() const;

		void set_data_path(const godot::String &p_path);
		godot::String get_data_path() const;

		void set_data_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_data_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		// Backward-compat single-path setters: route through orthophoto_paths.
		void set_orthophoto_path(const godot::String &p_path);
		godot::String get_orthophoto_path() const;

		void set_orthophoto_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_orthophoto_paths() const;

		void set_skip_white_pixels(bool p_skip);
		bool get_skip_white_pixels() const;

		void set_elevation_db(godot::Ref<S3DElevationDB> p_db);
		godot::Ref<S3DElevationDB> get_elevation_db() const;
	};

}

#endif // S3D_TILE_MANAGER_H
