#ifndef S3D_SHADERS_H
#define S3D_SHADERS_H

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/core/class_db.hpp>

namespace s3d {

// The addon's shaders (shaders/*.gdshader). In a project the files load from
// res://addons/scenery3d/shaders/, so an edit shows at the next run. An
// exported game does not have them: the addon folder carries a .gdignore
// (godot-cpp, sources, tools) and the exporter packs nothing below it. The
// source of every shader is therefore compiled into the library
// (s3d_shaders_gen.h, written by the SConstruct) and used when the file is
// missing. ProVPilot 1.1.0 shipped with untextured trees and vertex-colour
// roads for this reason.
class S3DShaders : public godot::Object {
	GDCLASS(S3DShaders, godot::Object)

protected:
	static void _bind_methods();

public:
	// name without path and extension: "tree", "roads", "chunk".
	// A new Shader per call when built from the embedded source: callers keep it.
	static godot::Ref<godot::Shader> load(const godot::String &name);
	static godot::String embedded_code(const godot::String &name);
};

} // namespace s3d

#endif
