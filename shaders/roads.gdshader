// Procedural road and railway surface for the base tier (design §13.9,
// scripts/build_world_roads.py). One material serves every tile: the
// vertex colour carries the class colour (rgb) and the surface kind
// (alpha × 255), the UV carries metres (u across the ribbon, signed, + =
// right of travel; v along it) and UV2 the half width and the lane or
// track count, so markings, sleepers and rail heads are drawn in metres.
//   kind 1 one-way carriageway (edge lines, lane dashes, hard shoulder right)
//   kind 2 two-way trunk / primary (edge lines + dashed centre line)
//   kind 3 two-way secondary / tertiary (dashed centre line)
//   kind 4 link / ramp / one-lane street (edge lines only)
//   kind 5 two-way motorway on one way (median + two carriageways)
//   kind 6 junction surface (plain asphalt, UV = metres from the node)
//   kind 7 concrete structure (bridge piers, parapets, abutments)
//   kind 8 railway ballast bed (sleepers per track, gravel)
//   kind 10 grass, 11 forest floor, 12 field, 13 water, 14 building roof, 15 building wall
//   (scenery editor areas, UV = metres)
//   kind 9 rail (extruded section: steel head, rusty flanks)
shader_type spatial;
render_mode cull_disabled, depth_draw_opaque;

const float LANE_MARGIN = 0.5;     // paved verge outside the edge lines
const float SHOULDER = 2.5;        // hard shoulder of a one-way carriageway
const float TRACK_SPACING = 3.8;   // between track centres of a multi-track bed
const float SLEEPER_LEN = 2.6;

varying vec2 uv_m;
varying vec4 vcol;
varying vec2 uv_info;

float hash2(vec2 p) {
	return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
}

float vnoise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	float a = hash2(i), b = hash2(i + vec2(1.0, 0.0));
	float c = hash2(i + vec2(0.0, 1.0)), d = hash2(i + vec2(1.0, 1.0));
	return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void vertex() {
	uv_m = UV;
	vcol = COLOR;
	uv_info = UV2;
}

// 1 inside a line of half-width hw centred on x0, soft edge of `soft` metres
float line(float x, float x0, float hw, float soft) {
	return 1.0 - smoothstep(hw - soft, hw + soft, abs(x - x0));
}

// dashed lane line every `period` metres with `dash` metres painted
float dashed(float x, float s, float x0, float hw, float soft, float dash, float period) {
	return line(x, x0, hw, soft) * step(mod(s, period), dash);
}

// lane dashes between `lanes` lanes that span [x_left, x_right]
float lane_dashes(float x, float s, float x_left, float x_right, float lanes, float px) {
	float m = 0.0;
	float lane_w = (x_right - x_left) / max(lanes, 1.0);
	for (int l = 1; l < 6; l++) {
		if (float(l) >= lanes) {
			break;
		}
		m = max(m, dashed(x, s, x_left + lane_w * float(l), 0.075, px, 6.0, 12.0));
	}
	return m;
}

void fragment() {
	float x = uv_m.x;
	float s = uv_m.y;
	float half_w = uv_info.x;
	float count = max(uv_info.y, 1.0);
	int kind = int(vcol.a * 255.0 + 0.5);
	float px = max(fwidth(x), 0.02);   // ≈ one pixel in metres: soft line edges
	float n1 = vnoise(vec2(x, s) * 0.45);
	float n2 = vnoise(vec2(x, s) * 3.0);
	vec3 base = vcol.rgb;
	float rough = 0.95;
	float spec = 0.08;
	float metal = 0.0;
	float mark = 0.0;
	vec3 mark_col = vec3(0.88, 0.88, 0.84);

	if (kind == 7) {
		// concrete: light grey with a coarse pour texture
		base = vec3(0.62, 0.61, 0.58) * (0.9 + 0.2 * n1) * (0.95 + 0.1 * n2);
		rough = 0.9;
		spec = 0.1;
	} else if (kind == 10 || kind == 11 || kind == 12) {
		// land cover patches: grass, forest floor, field
		vec3 a = kind == 10 ? vec3(0.36, 0.52, 0.22) : (kind == 11 ? vec3(0.16, 0.30, 0.13) : vec3(0.55, 0.47, 0.28));
		vec3 b = kind == 10 ? vec3(0.46, 0.60, 0.26) : (kind == 11 ? vec3(0.22, 0.38, 0.16) : vec3(0.68, 0.58, 0.34));
		base = mix(a, b, n1) * (0.9 + 0.2 * n2);
		rough = 1.0;
		spec = 0.02;
	} else if (kind == 13) {
		// water: dark blue-green, glossy
		base = mix(vec3(0.10, 0.22, 0.30), vec3(0.14, 0.30, 0.38), n1);
		rough = 0.15;
		spec = 0.7;
	} else if (kind == 14) {
		// building roof: dark red-brown tiles
		base = mix(vec3(0.42, 0.24, 0.18), vec3(0.50, 0.30, 0.22), n2);
		rough = 0.85;
		spec = 0.1;
	} else if (kind == 15) {
		// building wall: plaster, floor bands every 3 m (UV.y = height)
		float band = 1.0 - smoothstep(0.35, 0.45, abs(mod(uv_m.y, 3.0) - 1.5));
		base = mix(vec3(0.80, 0.78, 0.72), vec3(0.55, 0.60, 0.66), band * 0.5);
		rough = 0.8;
		spec = 0.15;
	} else if (kind == 8) {
		// ballast: dark grey-brown gravel; concrete sleepers 2.6 m long,
		// 0.26 m wide every 0.6 m under each track of the bed
		base = mix(vec3(0.30, 0.28, 0.25), vec3(0.42, 0.39, 0.35), n2) * (0.85 + 0.3 * n1);
		float sleeper = 1.0 - smoothstep(0.11, 0.13 + px, abs(mod(s, 0.6) - 0.3));
		float under_track = 0.0;
		for (int t = 0; t < 4; t++) {
			if (float(t) >= count) {
				break;
			}
			float c = (float(t) - (count - 1.0) * 0.5) * TRACK_SPACING;
			under_track = max(under_track, 1.0 - smoothstep(SLEEPER_LEN * 0.5 - px, SLEEPER_LEN * 0.5 + px, abs(x - c)));
		}
		base = mix(base, vec3(0.44, 0.43, 0.41) * (0.9 + 0.2 * n2), sleeper * under_track);
		rough = 1.0;
		spec = 0.03;
	} else if (kind == 9) {
		// rail: polished steel head (|x| < 36 mm), rusty flanks and foot
		float head = 1.0 - smoothstep(0.030, 0.036 + px, abs(x));
		base = mix(vec3(0.36, 0.25, 0.18) * (0.8 + 0.4 * n2), vec3(0.62, 0.62, 0.64), head);
		rough = mix(0.9, 0.3, head);
		metal = 0.7 * head;
		spec = mix(0.2, 0.6, head);
	} else {
		// asphalt with fine grain
		base = base * (0.85 + 0.3 * n1) * (0.92 + 0.16 * n2);
		if (kind == 1) {
			// one-way carriageway: hard shoulder on the right, lanes to its left
			float right_edge = half_w - LANE_MARGIN - SHOULDER;
			float left_edge = -half_w + LANE_MARGIN;
			mark = max(mark, line(x, left_edge, 0.075, px));
			mark = max(mark, line(x, right_edge, 0.10, px));
			mark = max(mark, lane_dashes(x, s, left_edge, right_edge, count, px));
			// shoulder slightly darker
			base *= 1.0 - 0.08 * step(right_edge + 0.2, x);
		} else if (kind == 5) {
			// two carriageways with a 3 m grass median
			float ax = abs(x);
			if (ax < 1.5) {
				base = mix(vec3(0.30, 0.36, 0.20), vec3(0.36, 0.40, 0.22), n1);
				rough = 1.0;
			} else {
				float inner = 1.5 + LANE_MARGIN;
				float outer = half_w - LANE_MARGIN - SHOULDER;
				mark = max(mark, line(ax, inner, 0.075, px));
				mark = max(mark, line(ax, outer, 0.10, px));
				mark = max(mark, lane_dashes(ax, s, inner, outer, count, px));
			}
		} else if (kind == 2) {
			mark = max(mark, dashed(x, s, 0.0, 0.075, px, 4.5, 9.0));
			mark = max(mark, line(abs(x), half_w - LANE_MARGIN, 0.075, px));
		} else if (kind == 3) {
			mark = max(mark, dashed(x, s, 0.0, 0.06, px, 3.0, 9.0));
		} else if (kind == 4) {
			mark = max(mark, line(abs(x), half_w - LANE_MARGIN, 0.06, px));
		}
		// kind 6: no markings
	}
	ALBEDO = mix(base, mark_col, mark * 0.9);
	ROUGHNESS = rough;
	SPECULAR = spec;
	METALLIC = metal;
}
