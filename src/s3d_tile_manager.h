#ifndef S3D_TILE_MANAGER_H
#define S3D_TILE_MANAGER_H

#include <godot_cpp/classes/node3d.hpp>

namespace godot
{

	// Manages the lifecycle of terrain tiles: loading, unloading, and LOD transitions.
	// Will use background threads for async tile loading to avoid frame hitches.
	class S3DTileManager : public Node3D
	{
		GDCLASS(S3DTileManager, Node3D);

	private:
		// Maximum number of tiles to load per frame to avoid stalls.
		int load_budget = 4;

		// Number of tiles beyond the load radius before unloading.
		int unload_margin = 2;

	protected:
		static void _bind_methods();

	public:
		S3DTileManager();
		~S3DTileManager();

		void _process(double delta) override;

		// Determine which tiles need loading/unloading based on camera position.
		void update_tiles(Vector3 camera_pos);

		void set_load_budget(int p_budget);
		int get_load_budget() const;

		void set_unload_margin(int p_margin);
		int get_unload_margin() const;
	};

}

#endif // S3D_TILE_MANAGER_H
