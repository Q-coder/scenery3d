
env = SConscript('godot-cpp/SConstruct')

# Paths
godot_headers_path = "godot-cpp/godot_headers/"
cpp_bindings_path = "godot-cpp/"

env.Append(CPPPATH="src/")
env.Append(CPPPATH="godot-cpp/include/")
env.Append(CPPPATH=['.', godot_headers_path, cpp_bindings_path + 'include/', cpp_bindings_path + 'include/core/', cpp_bindings_path + 'include/gen/'])
env.Append(LIBPATH=[cpp_bindings_path + 'bin/'])

src = Glob("src/*.cpp")

libpath = 'demo/bin/libscenery3d{}{}'.format(env['suffix'], env['SHLIBSUFFIX'])
sharedlib = env.SharedLibrary(libpath, src)
Default(sharedlib)
