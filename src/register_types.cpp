#include "register_types.h"
#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include "scenery3d.h"
#include "s3d_tile_manager.h"
#include "s3d_tile.h"
#include "s3d_elevation_db.h"
#include "s3d_coords.h"
#include "s3d_building_manager.h"
#include "s3d_water_manager.h"
#include "s3d_road_manager.h"

using namespace godot;

void initialize_scenery3d_module(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
	{
		return;
	}
	ClassDB::register_class<Scenery3D>();
	ClassDB::register_class<S3DTileManager>();
	ClassDB::register_class<S3DTile>();
	ClassDB::register_class<S3DElevationDB>();
	ClassDB::register_class<S3DCoords>();
	ClassDB::register_class<S3DBuildingManager>();
	ClassDB::register_class<S3DWaterManager>();
	ClassDB::register_class<S3DRoadManager>();
}

void uninitialize_scenery3d_module(ModuleInitializationLevel p_level)
{
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE)
	{
		return;
	}
}

extern "C"
{
	GDExtensionBool GDE_EXPORT scenery3d_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization)
	{
		godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

		init_obj.register_initializer(initialize_scenery3d_module);
		init_obj.register_terminator(uninitialize_scenery3d_module);
		init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

		return init_obj.init();
	}
}
