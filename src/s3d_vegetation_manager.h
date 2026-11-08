#ifndef S3D_VEGETATION_MANAGER_H
#define S3D_VEGETATION_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/classes/ref.hpp>

#include <unordered_map>
#include "s3d_frame.h"
#include "s3d_exclusion.h"
#include "s3d_elevation_db.h"
#include <unordered_set>
#include <vector>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>

namespace s3d
{

	// Streams swissTLM3D-derived per-tile tree positions and renders them
	// using `MultiMeshInstance3D`. One MultiMesh per terrain tile, sharing
	// a single user-supplied tree mesh (`tree_mesh`). Loads the .bin files
	// produced by `tools/extract_tlm_forests.py` on background threads.
	class S3DVegetationManager : public godot::Node3D
	{
		GDCLASS(S3DVegetationManager, godot::Node3D);

	private:
		// One tree instance, parsed from the .bin record format.
		struct TreeInstance {
			float dx, dz;          // Offset from tile SW corner, m.
			float ground_z;        // m ASL.
			float height_m;
			float yaw_rad;
			float crown_radius_m;
			int8_t species_hint = -1;   // forest_*.bin leaf type: 0 broadleaf, 1 conifer, -1 by altitude
		};
		// forest_{E}_{N}.bin (FOR1, scripts/build_world_forests.py): OSM forest
		// polygons the worker fills with trees at load time, and road / rail
		// corridors (centre line + half width) no tree may stand in. The
		// corridors of every forest file of a square also thin the surveyed
		// vegetation_*.bin trees of that square (roads clear inside forests).
		struct Corridor {
			std::vector<float> xz;   // dx, dz pairs along the centre line
			float half_width = 3.0f;
			float min_x = 0, min_z = 0, max_x = 0, max_z = 0;
		};
		struct ForestPolygon {
			int kind = 2;            // 0 broadleaf, 1 conifer, 2 mixed
			float density = 1.0f / 36.0f;   // trees per m2
			float height_m = 20.0f;
			std::vector<std::vector<float>> rings;   // outer first, then holes; dx, dz pairs
		};
		static bool parse_forest_file(const std::string &path, std::vector<ForestPolygon> &polys, std::vector<Corridor> &corridors, double &tile_east, double &tile_north);
		static void fill_polygons(const std::vector<ForestPolygon> &polys, uint32_t seed, std::vector<TreeInstance> &out);
		static void apply_corridors(const std::vector<Corridor> &corridors, std::vector<TreeInstance> &trees);

		struct LoadResult {
			std::string tile_id;
			bool success = false;
			bool forest_fill_skipped = false;
			double tile_east = 0;
			double tile_north = 0;
			std::vector<TreeInstance> trees;
		};

		struct LoadRequest {
			std::string tile_id;
			std::string path;
			bool forest = false;                      // a forest_*.bin: fill polygons
			bool trees_disabled = false;              // a surveyed tile covers the square: corridors only
			std::vector<std::string> corridor_paths;  // every forest file of the square (all groups)
		};

		struct TileState {
			godot::MultiMeshInstance3D *node = nullptr;      // every kept tree (card mode: the near set)
			godot::MultiMeshInstance3D *far_node = nullptr;  // card mode: the thinned far set
			bool loading = false;
			bool loaded = false;
			bool no_data = false;
			// CPU-side statistics of the near set (tests; the headless renderer keeps no instance data)
			std::vector<int> species_counts;
			float min_height = 0.0f, max_height = 0.0f;
			int low_conifers = 0, high_conifers = 0;   // species picks below 700 m / above 1 300 m
			int low_total = 0, high_total = 0;
			int alpine_total = 0;                       // above 1 900 m
			float low_height_sum = 0.0f, alpine_height_sum = 0.0f;
			float far_height_ratio = 0.0f;
			int snapped = 0;                 // trees placed on the terrain's height instead of the record's
			float ground_min = 0.0f, ground_max = 0.0f;
			bool snap_pending = false;       // terrain not loaded yet when built: rebuild when it arrives
			bool reloading = false;          // a rebuild is in flight; the old nodes stay until it lands
			bool forest_fill_skipped = false;   // a forest file whose square has a surveyed tile
			std::vector<float> positions;    // dx, dz of the near set when debug_keep_positions (tests)
		};

		// vegetation_{E}_{N}.bin index per configured directory.
		struct ManifestEntry {
			std::string path;
			int tile_e = 0;
			int tile_n = 0;
			bool forest = false;                      // forest_*.bin (polygons) rather than vegetation_*.bin (trees)
			bool trees_disabled = false;              // a vegetation_*.bin of the same square exists: corridors only
			std::vector<std::string> corridor_paths;
		};

		std::vector<std::vector<ManifestEntry>> manifests;  // by group
		bool manifest_loaded = false;

		// Tile state keyed by "<group_index>:<tile_e>_<tile_n>".
		std::unordered_map<std::string, TileState> tiles;

		// Zones of higher-priority packs that must stay free of trees.
		S3DExclusionZones exclusions;
		godot::Array exclusion_zones;
		int excluded_count = 0;
		void clear_loaded_tiles();

		// Background worker.
		static constexpr int NUM_WORKERS = 2;
		std::vector<std::thread> worker_threads;
		std::mutex work_mutex;
		std::condition_variable work_cv;
		std::deque<LoadRequest> work_queue;
		std::mutex results_mutex;
		std::deque<LoadResult> results_queue;
		std::atomic<bool> worker_running{false};

		// Configuration.
		godot::PackedStringArray vegetation_paths;
		godot::Ref<godot::Mesh> tree_mesh;          // The mesh used per instance.
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
		S3DFrame frame;  // world frame (flat legacy / ECEF), origin_* mirror into it
		int load_radius_m = 4000;
		int unload_margin_m = 1000;
		double scale_jitter = 0.20;   // ±20 % per-instance scale jitter.
		double height_to_mesh_unit = 1.0;  // Multiplier from "tree_height_m"
		                                   // to mesh scale Y. If your mesh is
		                                   // 1 m tall, leave at 1.0; if 10 m
		                                   // tall, set to 0.1.
		// Pre-rotation around the X axis applied to every instance, degrees.
		// Use 90 for Blender/glTF assets exported with Z-up so trunks point
		// up in Godot's Y-up world.
		double mesh_pitch_deg = 0.0;
		// Fraction of trees per tile to actually render (0..1). Selection is
		// deterministic per tile, so toggling at runtime gives a stable subset.
		double density_fraction = 1.0;
		// ── Card mode (docs/WORLD_SCENERY_DESIGN.md §14) ───────────────────
		// Instead of `tree_mesh`, every tree is three vertical quads and a
		// canopy quad (8 triangles) textured from a species atlas (columns =
		// species, upper row side view, lower row top view) through
		// shaders/tree.gdshader. Per tile two MultiMeshes: the near set
		// (every kept tree, visible to near_range_m) and the far set (every
		// 1/far_fraction-th tree at far_scale, near_range_m .. far_range_m).
		bool card_mode = false;
		godot::String atlas_path = "res://trees/tree_atlas.png";
		godot::String atlas_meta_path = "res://trees/tree_atlas.json";
		double near_range_m = 2500.0;
		double far_range_m = 8000.0;
		double far_fraction = 0.25;
		double far_scale = 1.6;
		int results_per_frame = 3;
		bool debug_keep_positions = false;
		// The host's terrain heights (Scenery3D.get_elevation_db()): in card
		// mode every tree stands on the terrain that is actually drawn, not
		// on the record's ground height from another DEM (SwissTopo 1 m
		// records over the SRTM base tier sat metres under the surface).
		godot::Ref<S3DElevationDB> elevation_db;
		bool snap_to_terrain = true;
		uint64_t snap_epoch = 0;
		double wind_speed_mps = 0.0;
		void resnap_pending_tiles();
		struct Species {
			std::string name;
			bool conifer = false;
			double height_m = 20.0;
			double crown_ratio = 0.6;   // crown width / height of the card
		};
		std::vector<Species> species;
		godot::Ref<godot::Mesh> card_mesh;
		godot::Ref<godot::ShaderMaterial> card_material;
		bool card_assets_ready = false;
		void prepare_card_assets();
		int pick_species(uint32_t hash, float ground_z, int hint = -1) const;
		const TileState *find_tile_state(int tile_e, int tile_n) const;
		godot::MultiMeshInstance3D *make_instances(const LoadResult &result, const std::vector<size_t> &indices, double scale_mul, const char *suffix, TileState *stats, bool far_set);
		// Near / far hand-over band: both sets are drawn within lod_band_m of
		// near_range_m and the shader crossfades them per tree by dither, so no
		// tile swaps as a whole (gery 2026-11-08: "visible flickering of chunks").
		double lod_band_m = 400.0;

		// Last camera position.
		double last_cam_e = 0;
		double last_cam_n = 0;

		void start_worker();
		void stop_worker();
		void worker_func();

		void load_manifests();
		void update_tiles(godot::Vector3 camera_pos);
		void process_load_results();

	protected:
		static void _bind_methods();

	public:
		S3DVegetationManager();
		~S3DVegetationManager();

		void _process(double delta) override;

		void set_vegetation_paths(const godot::PackedStringArray &p_paths);
		godot::PackedStringArray get_vegetation_paths() const;

		void set_tree_mesh(const godot::Ref<godot::Mesh> &p_mesh);
		godot::Ref<godot::Mesh> get_tree_mesh() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;
		void set_origin_north(double p_north);
		double get_origin_north() const;
		void set_frame_mode(int p_mode);
		int get_frame_mode() const { return frame.mode; }

		void set_load_radius_m(int p_r);
		int get_load_radius_m() const;

		void set_scale_jitter(double p_v);
		double get_scale_jitter() const;

		void set_height_to_mesh_unit(double p_v);
		double get_height_to_mesh_unit() const;

		void set_mesh_pitch_deg(double p_v);
		double get_mesh_pitch_deg() const;

		void set_density_fraction(double p_v);
		double get_density_fraction() const;

		int get_active_tile_count() const;
		void set_card_mode(bool on);
		void set_elevation_db(const godot::Ref<S3DElevationDB> &db) { elevation_db = db; }
		godot::Ref<S3DElevationDB> get_elevation_db() const { return elevation_db; }
		void set_snap_to_terrain(bool on) { snap_to_terrain = on; clear_loaded_tiles(); }
		// Sway amplitude from the weather's wind: none in calm air, full at 12 m/s.
		void set_wind_speed_mps(double v);
		double get_wind_speed_mps() const { return wind_speed_mps; }
		bool get_snap_to_terrain() const { return snap_to_terrain; }
		bool get_card_mode() const { return card_mode; }
		void set_atlas_path(const godot::String &p);
		godot::String get_atlas_path() const { return atlas_path; }
		void set_atlas_meta_path(const godot::String &p);
		godot::String get_atlas_meta_path() const { return atlas_meta_path; }
		void set_near_range_m(double v);
		double get_near_range_m() const { return near_range_m; }
		void set_far_range_m(double v);
		double get_far_range_m() const { return far_range_m; }
		void set_far_fraction(double v);
		double get_far_fraction() const { return far_fraction; }
		void set_far_scale(double v);
		double get_far_scale() const { return far_scale; }
		int get_species_count() { prepare_card_assets(); return (int)species.size(); }
		// The card mesh (8 triangles, unit height, unit crown width) — tests.
		godot::Ref<godot::Mesh> get_card_mesh();
		// Per-tile instance counts of the tile at grid (east, north): [near, far]; empty when not loaded.
		godot::PackedInt32Array tile_instance_counts(int tile_e, int tile_n) const;
		// {near, far, species: [count per species], min_height, max_height, low_conifers, low_total, high_conifers, high_total, far_height_ratio}
		godot::Dictionary tile_stats(int tile_e, int tile_n) const;
		// Loaded tiles still waiting for terrain heights (or rebuilding).
		int pending_tile_count() const;
		void set_debug_keep_positions(bool on) { debug_keep_positions = on; }
		bool get_debug_keep_positions() const { return debug_keep_positions; }
		// dx, dz pairs of the near set of the tile (debug_keep_positions on).
		godot::PackedVector2Array tile_tree_positions(int tile_e, int tile_n) const;

		void set_exclusion_zones(const godot::Array &zones);
		godot::Array get_exclusion_zones() const;
		int get_excluded_tree_count() const;
	};

}

#endif // S3D_VEGETATION_MANAGER_H
