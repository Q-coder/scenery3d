#ifndef SCENERY3D_H
#define SCENERY3D_H

#include <godot_cpp/classes/node3d.hpp>

namespace godot
{

	// Main entry point for the terrain system.
	// Owns the tile manager and elevation database, and coordinates
	// tile loading around the camera position.
	class Scenery3D : public Node3D
	{
		GDCLASS(Scenery3D, Node3D);

	private:
		int tile_size = 1024;
		int load_radius = 8;
		String data_path;

	protected:
		static void _bind_methods();

	public:
		Scenery3D();
		~Scenery3D();

		void _ready() override;
		void _process(double delta) override;

		void set_tile_size(int p_size);
		int get_tile_size() const;

		void set_load_radius(int p_radius);
		int get_load_radius() const;

		void set_data_path(const String &p_path);
		String get_data_path() const;
	};

}

#endif // SCENERY3D_H
