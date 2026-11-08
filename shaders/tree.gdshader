// Forest tree cards (docs/WORLD_SCENERY_DESIGN.md §14): S3DVegetationManager
// draws every tree as three vertical quads and a canopy quad, one shared
// material, alpha-tested from a species atlas (columns = species, upper half
// the side view, lower half the top view). INSTANCE_CUSTOM: x = species
// column, y / z = per-tree variation. COLOR.r = 1 on the canopy quad. The
// canopy quad fades in as the view steepens (no cross seen from above), the
// side quads fade out when looked at from straight above; beyond
// fade_begin_m the trees dissolve by an ordered dither before the far set is
// culled. Double build: the camera is taken in model space from
// MODELVIEW_MATRIX, never from a world-space position.
shader_type spatial;
render_mode cull_disabled, depth_draw_opaque, specular_disabled;

uniform sampler2D atlas : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform int species_count = 4;
uniform float fade_begin_m = 6400.0;
uniform float fade_end_m = 8000.0;
uniform float wind = 0.015;
uniform float alpha_cut : hint_range(0.1, 0.9) = 0.45;

varying float v_top;
varying float v_view_up;
varying float v_dist;
varying vec3 v_tint;

void vertex() {
	int sp = clamp(int(INSTANCE_CUSTOM.x + 0.5), 0, max(species_count - 1, 0));
	float uw = 1.0 / float(max(species_count, 1));
	v_top = COLOR.r;
	UV = vec2((float(sp) + UV.x) * uw, (v_top > 0.5 ? 0.5 : 0.0) + UV.y * 0.5);
	// camera in instance space; undo the per-axis instance scale for the angle
	vec3 cam_l = (inverse(MODELVIEW_MATRIX) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
	vec3 sc = vec3(length(MODEL_MATRIX[0].xyz), length(MODEL_MATRIX[1].xyz), length(MODEL_MATRIX[2].xyz));
	vec3 to_cam = normalize((cam_l - vec3(0.0, 0.5, 0.0)) * sc);
	v_view_up = abs(to_cam.y);
	// a little sway at the top of the card
	float ph = TIME * 1.1 + float(INSTANCE_ID) * 0.37;
	VERTEX.x += sin(ph) * wind * VERTEX.y;
	VERTEX.z += cos(ph * 0.83) * wind * VERTEX.y;
	v_dist = length((MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz);
	v_tint = mix(vec3(1.0), vec3(1.06, 0.98, 0.82), INSTANCE_CUSTOM.y) * (0.82 + 0.3 * INSTANCE_CUSTOM.z);
	NORMAL = vec3(0.0, 1.0, 0.0);
}

void fragment() {
	vec4 c = texture(atlas, UV);
	float a = c.a;
	if (v_top > 0.5) {
		a *= smoothstep(0.3, 0.6, v_view_up);
	} else {
		a *= 1.0 - smoothstep(0.82, 0.97, v_view_up);
	}
	float keep = 1.0 - smoothstep(fade_begin_m, fade_end_m, v_dist);
	float dither = fract(52.9829189 * fract(dot(FRAGCOORD.xy, vec2(0.06711056, 0.00583715))));
	if (keep < dither) {
		a = 0.0;
	}
	ALBEDO = c.rgb * v_tint;
	ALPHA = a;
	ALPHA_SCISSOR_THRESHOLD = alpha_cut;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}
