#ifndef S3D_VEGETATION_MANAGER_H
#define S3D_VEGETATION_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
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

	// Streams swissTLM3D-derived per-tile tree positions and renders them
	// using `MultiMeshInstance3D`. One MultiMesh per terrain tile, sharing
	// a single user-supplied tree mesh (`tree_mesh`). Loads the .bin files
	// produced by `tools/extract_tlm_forests.py` on background threads.
	class S3DVegetationManager : public Node3D
	{
		GDCLASS(S3DVegetationManager, Node3D);

	private:
		// One tree instance, parsed from the .bin record format.
		struct TreeInstance {
			float dx, dz;          // Offset from tile SW corner, m.
			float ground_z;        // m ASL.
			float height_m;
			float yaw_rad;
			float crown_radius_m;
		};

		struct LoadResult {
			std::string tile_id;
			bool success = false;
			double tile_east = 0;
			double tile_north = 0;
			std::vector<TreeInstance> trees;
		};

		struct LoadRequest {
			std::string tile_id;
			std::string path;
		};

		struct TileState {
			MultiMeshInstance3D *node = nullptr;
			bool loading = false;
			bool loaded = false;
			bool no_data = false;
		};

		// vegetation_{E}_{N}.bin index per configured directory.
		struct ManifestEntry {
			std::string path;
			int tile_e = 0;
			int tile_n = 0;
		};

		std::vector<std::vector<ManifestEntry>> manifests;  // by group
		bool manifest_loaded = false;

		// Tile state keyed by "<group_index>:<tile_e>_<tile_n>".
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

		// Configuration.
		PackedStringArray vegetation_paths;
		Ref<Mesh> tree_mesh;          // The mesh used per instance.
		double origin_east = 2600000.0;
		double origin_north = 1200000.0;
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

		// Last camera position.
		double last_cam_e = 0;
		double last_cam_n = 0;

		void start_worker();
		void stop_worker();
		void worker_func();

		void load_manifests();
		void update_tiles(Vector3 camera_pos);
		void process_load_results();

	protected:
		static void _bind_methods();

	public:
		S3DVegetationManager();
		~S3DVegetationManager();

		void _process(double delta) override;

		void set_vegetation_paths(const PackedStringArray &p_paths);
		PackedStringArray get_vegetation_paths() const;

		void set_tree_mesh(const Ref<Mesh> &p_mesh);
		Ref<Mesh> get_tree_mesh() const;

		void set_origin_east(double p_east);
		double get_origin_east() const;
		void set_origin_north(double p_north);
		double get_origin_north() const;

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
	};

}

#endif // S3D_VEGETATION_MANAGER_H
