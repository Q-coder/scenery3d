#include "s3d_shaders.h"

#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "s3d_shaders_gen.h"

using namespace godot;

namespace s3d {

void S3DShaders::_bind_methods()
{
	ClassDB::bind_static_method("S3DShaders", D_METHOD("load", "name"), &S3DShaders::load);
	ClassDB::bind_static_method("S3DShaders", D_METHOD("embedded_code", "name"), &S3DShaders::embedded_code);
}

String S3DShaders::embedded_code(const String &name)
{
	for (int i = 0; i < S3D_EMBEDDED_SHADER_COUNT; i++) {
		if (String(S3D_EMBEDDED_SHADERS[i].name) == name) {
			return String::utf8(S3D_EMBEDDED_SHADERS[i].code);
		}
	}
	return String();
}

Ref<Shader> S3DShaders::load(const String &name)
{
	const String path = String("res://addons/scenery3d/shaders/") + name + ".gdshader";
	ResourceLoader *rl = ResourceLoader::get_singleton();
	if (rl && rl->exists(path)) {
		Ref<Shader> sh = rl->load(path);
		if (sh.is_valid()) return sh;
	}
	const String code = embedded_code(name);
	if (code.is_empty()) {
		UtilityFunctions::push_warning("S3DShaders: no shader named ", name);
		return Ref<Shader>();
	}
	Ref<Shader> sh;
	sh.instantiate();
	sh->set_code(code);
	return sh;
}

} // namespace s3d
