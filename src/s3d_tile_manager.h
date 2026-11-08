#ifndef S3D_TILE_MANAGER_H
#define S3D_TILE_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include "s3d_terrain_patches.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <climits>
#include <cstdint>

#include "s3d_elevation_db.h"
#include "s3d_frame.h"

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
			// Chunks only: composite resolution the mesh was built at
			// (chunk_res_for_distance). A different desired value queues a
			// rebuild so chunks sharpen as the camera approaches.
			int chunk_res = 0;
		};

		// Background file load request.
		struct LoadRequest {
			int ei, ni;
			uint64_t key;
			// Candidate file paths tried in order (first existing/openable wins).
			std::vector<std::string> paths;
			// Pixels-per-tile override per candidate path, parallel to `paths`
			// (0 = use tile_px). Filled from each data path's manifest.json so
			// mixed-resolution regions coexist in one manager instance.
			std::vector<int> path_px;
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
			// Land-cover candidates (forest_E_N.bin per vegetation dir): the
			// worker paints the first that parses into the tile texture when
			// no orthophoto covers the tile (design §14.6).
			std::vector<std::string> landcover_paths;

			// Chunk fields (is_chunk == true).
			bool is_chunk = false;
			int chunk_grid_ei = 0;
			int chunk_grid_ni = 0;
			int chunk_tile_count = 0;
			int composite_res = 0;
			// Candidate base directories for chunk tile lookup.
			std::vector<std::string> base_paths;
			// Pixels-per-tile override per base directory, parallel to
			// base_paths (0 = use tile_px). See LoadRequest::path_px.
			std::vector<int> base_path_px;
			// Orthophoto mip directories for chunk compositing
			// (one per orthophoto root; first existing JPEG per sub-tile wins).
			std::vector<std::string> ortho_mip_dirs;
			// Vegetation dirs for the chunk's land-cover fallback: every
			// sub-tile without a mip3 JPEG is painted from its forest file
			// (base colour when it has none).
			std::vector<std::string> landcover_dirs;
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
			// Same, pre-composited across sources in the worker (no-data
			// pixels filled from lower-priority sources): raw RGB8, w² px.
			struct RawRgb { int w = 0; std::vector<uint8_t> rgb; };
			std::unordered_map<int, RawRgb> chunk_ortho_rgb;
			int chunk_tile_count = 0;
			// Some sub-tile was painted from land cover: the chunk is kept
			// however few pixels are "real" (a coastal chunk with a few
			// squares of land is land, not an orthophoto gap).
			bool landcover_painted = false;
		};

		// Active tile states keyed by tile_key(ei, ni).
		std::unordered_map<uint64_t, TileState> tiles;
		// Scenery-editor terrain patches (plateaus, corridors) written into every
		// close tile's heights as it is decoded; set_terrain_patches reloads the tiles.
		S3DTerrainPatches terrain_patches;

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

		// Ordered list of vegetation directories (forest_E_N.bin, FOR1).
		// Where no orthophoto covers a tile the worker rasterises the
		// square's forest polygons into a small ground texture, so woods
		// stay recognisable past the tree cards, out to the far chunks.
		godot::PackedStringArray landcover_paths;

		// Far chunks are ShaderMaterials (shaders/chunk.gdshader): a chunk
		// straddling the close ring gets a mask of the squares whose
		// individual tile is drawn and discards its fragments there, so it
		// can never surface through a tile (the 30 m sink alone fails where
		// a LOD-reduced tile bridges a ravine). The mask is rebuilt when the
		// resident geometry or the camera square changes.
		godot::Ref<godot::Shader> chunk_shader;
		godot::Ref<godot::ImageTexture> cover_tex;
		godot::PackedByteArray cover_px;   // the mask bytes (row = ni, col = ei), kept for chunk_cover_covers
		int cover_e_min = 0, cover_n_min = 0, cover_n = 0;
		uint64_t cover_gen_built = ~0ull;
		int cover_cam_ei = INT32_MIN, cover_cam_ni = INT32_MIN;
		std::unordered_set<uint64_t> masked_chunks;
		godot::Ref<godot::Material> make_chunk_material(int cg_ei, int cg_ni, godot::Ref<godot::Texture2D> p_tex);
		void update_chunk_cover(int cam_ei, int cam_ni, int max_radius);

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
		// Candidate forest files (one per landcover path) for a tile.
		std::vector<std::string> landcover_paths_for_tile(int ei, int ni) const;

		// pixels_per_tile declared by <dir>/manifest.json, or 0 if the
		// manifest is missing / has no such field (→ node-wide tile_px).
		static int manifest_pixels_per_tile(const std::string &p_dir);
		// Effective pixels per tile for one data path: the manifest override
		// when present, else the node-wide value.
		int tile_px_for_path(const std::string &p_path) const;
		// "crs" of a terrain manifest as an EPSG code (0 when absent).
		static int manifest_crs_epsg(const std::string &p_dir);
		// Re-parse every data path's manifest after data_paths changes.
		void rebuild_path_tile_px();

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
		// Coverage-query state for the coarse-globe suppression pass (Phase 2):
		// last camera tile / chunk-grid indices from update_tiles(), and a
		// generation counter that bumps whenever resident geometry appears or
		// disappears, so the host rebuilds its suppression mask only when
		// coverage actually changed.
		int last_cam_ei = 0;
		int last_cam_ni = 0;
		int last_cam_cg_ei = 0;
		int last_cam_cg_ni = 0;
		uint64_t coverage_generation = 0;
		// Ordered list of directories to search for tile files.
		// The first entry is returned by get_data_path() for backward compat.
		godot::PackedStringArray data_paths;
		godot::PackedStringArray data_paths_requested;  // last set_data_paths() argument (before CRS filtering)

		// Per-data-path pixels-per-tile overrides, keyed by the exact path
		// string in data_paths. Populated from each path's manifest.json
		// ("pixels_per_tile") — this is what lets a data-light region
		// (e.g. Norway at 34 px/tile) stream alongside CH/BW (512/1024 px)
		// in one manager. Paths without an override fall back to tile_px.
		std::unordered_map<std::string, int> path_tile_px;

		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		// World frame (flat legacy or ECEF, design §9.5); origin_* mirror
		// into it. Tiles/chunks are placed through frame.place() /
		// S3DTile::set_placement() and the camera is mapped to grid E/N
		// through frame.world_to_grid().
		S3DFrame frame;

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

		// Resolution of chunk composite heightmaps by distance (tiles from
		// the camera to the chunk's nearest tile): 129 px (62 m) near the
		// individual-tile ring, 65 px (125 m) mid, 33 px (250 m) far. The
		// near value matches the 30 m base packs' outermost tile LOD so the
		// tile/chunk handoff no longer changes the mesh spacing 4×.
		static constexpr int CHUNK_COMPOSITE_RES = 33;
		static constexpr int CHUNK_RES_NEAR = 129;
		static constexpr int CHUNK_RES_MID = 65;
		static constexpr int CHUNK_NEAR_TILES = 32;
		static constexpr int CHUNK_MID_TILES = 64;
		static constexpr int CHUNK_RES_HYSTERESIS_TILES = 4;
		int chunk_res_for_distance(int dist, int current_res) const;

		// Y offset (metres) applied to chunk meshes so they sit just below
		// the individual-tile surface and lose the depth test wherever the
		// two overlap. Scaled with the composite spacing: 30 m at 250 m
		// spacing (the error of 250 m sampling vs 1 m tiles on typical
		// terrain), 7.5 m at 62 m, so the surface no longer jumps 30 m when
		// individual tiles replace a fine chunk.
		static constexpr double CHUNK_Y_BIAS = -30.0;
		static constexpr double CHUNK_Y_BIAS_REF_SPACING_M = 250.0;

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

		// Bump the coverage generation (resident geometry appeared/gone).
		void bump_coverage();
		// A chunk-aligned cell is covered when its far-zone chunk is resident,
		// or every sub-tile in it has a mesh (close-ring cells have no chunk).

	protected:
		static void _bind_methods();

	public:
		S3DTileManager();
		~S3DTileManager();

		void _process(double delta) override;

		void update_tiles(godot::Vector3 camera_pos);
		int get_active_tile_count() const;
		// Tiles + chunks still loading (in flight on a worker or queued) —
		// drives the intro-zoom "tiles loading" progress bar in the host.
		int get_pending_count();

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

		// --- Coverage mask for the coarse-globe suppression pass (Phase 2) ---
		// One byte per TILE (tile_size metres), row-major, (2r+1)^2 entries
		// with r = get_coverage_mask_radius(), centred on the camera tile
		// (get_cam_tile_ei/ni): 255 where resident geometry covers the tile
		// (a loaded far-zone chunk stamps its whole chunk_size x chunk_size
		// footprint; a loaded individual tile stamps itself), 0 elsewhere. The
		// host uploads it verbatim as the globe's per-fragment mask, so the
		// globe is hidden exactly under resident geometry and stays visible in
		// unloaded / no-data tiles (no holes to space, no poke-through next to
		// a no-data tile at a region's data edge).
		godot::PackedByteArray get_coverage_mask() const;
		int get_coverage_mask_radius() const { return far_radius + unload_margin + chunk_size; }
		int get_cam_tile_ei() const { return last_cam_ei; }
		int get_cam_tile_ni() const { return last_cam_ni; }
		int get_chunk_size() const { return chunk_size; }
		// Bumps whenever resident geometry appears or disappears, so the host
		// can refresh its suppression mask only on actual changes.
		int get_coverage_generation() const { return (int)coverage_generation; }

		void set_data_path(const godot::String &p_path);
		godot::String get_data_path() const;

		void set_data_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_data_paths() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;

		void set_origin_north(double p_north);
		double get_origin_north() const;

		// 0 = flat legacy frame, 1 = ECEF (see S3DFrame).
		void set_frame_mode(int p_mode);
		int get_frame_mode() const { return frame.mode; }
		const S3DFrame &get_frame() const { return frame; }

		// Backward-compat single-path setters: route through orthophoto_paths.
		void set_orthophoto_path(const godot::String &p_path);
		godot::String get_orthophoto_path() const;

		void set_orthophoto_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_orthophoto_paths() const;

		void set_landcover_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_landcover_paths() const;
		// The land-cover raster of one forest file at px pixels (tests);
		// an empty Ref when the file does not parse.
		godot::Ref<godot::Image> landcover_image(const godot::String &p_path, int p_px) const;
		// Whether the chunk cover mask marks square (ei, ni) as drawn by an
		// individual tile (tests).
		bool chunk_cover_covers(int p_ei, int p_ni) const;
		int get_masked_chunk_count() const { return (int)masked_chunks.size(); }

		void set_skip_white_pixels(bool p_skip);
		bool get_skip_white_pixels() const;

		void set_elevation_db(godot::Ref<S3DElevationDB> p_db);
		godot::Ref<S3DElevationDB> get_elevation_db() const;
		// {"plateaus": [{"poly": PackedVector2Array (e, n), "h": float}],
		//  "corridors": [{"stations": PackedVector3Array (e, n, h), "reach": float}]}
		void set_terrain_patches(const godot::Dictionary &p_patches);
		int get_terrain_patch_count() const;
		// Drop every loaded close tile so it streams in again (patched).
		void reload_tiles();
		int get_reload_count() const { return reload_count; }
	private:
		int reload_count = 0;
	public:
	};

}

#endif // S3D_TILE_MANAGER_H
