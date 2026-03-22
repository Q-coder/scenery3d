#ifndef S3D_TILE_MANAGER_H
#define S3D_TILE_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>
#include <unordered_map>
#include <vector>
#include <utility>

namespace godot
{

	class S3DTile;

	// Manages the lifecycle of terrain tiles: loading, unloading, and LOD transitions.
	// Decides which tiles to load/unload based on camera position each frame.
	class S3DTileManager : public Node3D
	{
		GDCLASS(S3DTileManager, Node3D);

	private:
		// Currently loaded tiles, keyed by ((uint64_t)(uint32_t)tile_x << 32) | (uint32_t)tile_z.
		std::unordered_map<uint64_t, S3DTile *> active_tiles;

		// Tiles queued for loading (tile_x, tile_z).
		std::vector<std::pair<int, int>> pending_loads;

		// World size of each tile in units.
		int tile_size = 1024;

		// Radius in tiles around camera to keep loaded.
		int load_radius = 8;

		// Maximum number of tiles to load per frame to avoid stalls.
		int load_budget = 4;

		// Number of tiles beyond the load radius before unloading.
		int unload_margin = 2;

		// Path to tile data on disk.
		String data_path;

		// Helper to compute tile key from grid coordinates.
		static inline uint64_t tile_key(int tx, int tz) {
			return ((uint64_t)(uint32_t)tx << 32) | (uint32_t)tz;
		}

	protected:
		static void _bind_methods();

	public:
		S3DTileManager();
		~S3DTileManager();

		void _process(double delta) override;

		// Determine which tiles need loading/unloading based on camera position.
		void update_tiles(Vector3 camera_pos);

		// Number of currently active (loaded) tiles.
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
	};

}

#endif // S3D_TILE_MANAGER_H
