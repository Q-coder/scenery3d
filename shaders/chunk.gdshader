// Far terrain chunk (S3DTileManager, 8 x 8 squares at ~250 m spacing).
//
// A chunk straddling the close ring is drawn under the individual tiles,
// sunk 30 m; where a LOD-reduced tile bridges a ravine or a ridge the chunk
// still surfaces through it, which showed as 64 m blocks of the chunk's
// land-cover texture flickering around woods (gery, ENGM 2026-11-08). The
// tile manager hands every straddling chunk a mask of the squares whose
// individual tile is drawn; fragments inside such a square are discarded,
// so the chunk can never show through a drawn tile, whatever the geometry.
shader_type spatial;
render_mode cull_disabled;

uniform sampler2D albedo_tex : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform bool has_albedo_tex = false;
uniform vec4 albedo_color : source_color = vec4(0.45, 0.55, 0.35, 1.0);

// 255 where a close tile is drawn; pixel (x, y) = square (cover_rect.x + x, cover_rect.y + y).
uniform sampler2D cover_mask : filter_nearest, repeat_disable;
// x, y: square index of the mask's pixel (0, 0); z: mask size in squares (0 = no mask).
uniform vec4 cover_rect = vec4(0.0);
// The chunk's south-west square (ei, ni) and its size in squares.
uniform vec2 chunk_base = vec2(0.0);
uniform float chunk_tiles = 8.0;

void fragment() {
	if (cover_rect.z > 0.0) {
		// UV.x = 0 is the east edge, UV.y = 0 the south edge of the chunk.
		float de = clamp(floor((1.0 - UV.x) * chunk_tiles), 0.0, chunk_tiles - 1.0);
		float dn = clamp(floor(UV.y * chunk_tiles), 0.0, chunk_tiles - 1.0);
		vec2 sq = chunk_base + vec2(de, dn);
		vec2 m = (sq - cover_rect.xy + vec2(0.5)) / cover_rect.z;
		if (all(greaterThanEqual(m, vec2(0.0))) && all(lessThan(m, vec2(1.0)))) {
			if (texture(cover_mask, m).r > 0.5) {
				discard;
			}
		}
	}
	ALBEDO = has_albedo_tex ? texture(albedo_tex, UV).rgb : albedo_color.rgb;
	ROUGHNESS = 1.0;
	METALLIC = 0.0;
}
