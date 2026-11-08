#include "s3d_tile_manager.h"
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <algorithm>
#include "s3d_tile.h"
#include "s3d_forest_file.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/variant/vector4.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/json.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/time.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

namespace s3d {

// --- Orthophoto no-data test ---
// SWISSIMAGE encodes territory outside Switzerland as solid white in most
// border tiles but as solid BLACK in others (e.g. around the Jestetten
// enclave near Rafz, 78-90 % of a tile). Both must yield to a lower-priority
// source. JPEG compression leaves the flat areas a few counts off pure
// white/black, hence the tolerances (which also removed the thin white line
// that survived the exact ==255 test at the CH/DE border).
static inline bool ortho_is_nodata(uint8_t r, uint8_t g, uint8_t b)
{
	return (r >= 250 && g >= 250 && b >= 250) || (r <= 6 && g <= 6 && b <= 6);
}

// How much a pixel should take from the next source, 0..1. Pure no-data is
// 1; pixels *near* white or black blend partially. The on-disk mips (mip2 /
// mip3, used for distant tiles and the far chunks) were downsampled from
// tiles with white margins, so along the outer edge of the Swiss set their
// edge pixels are light averages of white and ground — not "white", yet a
// visible light staircase from altitude. Blending them toward the other
// source is safe: both orthophotos show the same ground.
static inline float ortho_nodata_weight(uint8_t r, uint8_t g, uint8_t b)
{
	const int mn = MIN(r, MIN(g, b));
	const int mx = MAX(r, MAX(g, b));
	if (mn >= 250 || mx <= 6) return 1.0f;
	if (mn >= 200) return (float)(mn - 200) / 50.0f;  // 200..250 -> 0..1
	if (mx <= 14) return (float)(14 - mx) / 8.0f;      // 14..6 -> 0..1
	return 0.0f;
}

// Merge `src` into `dst` (both RGB8, same size): no-data pixels of dst take
// src's pixel, near-no-data pixels blend toward it. Returns true when
// (near-)no-data pixels remain in dst.
static bool ortho_merge_nodata(PackedByteArray &dst, const PackedByteArray &src)
{
	uint8_t *cp = dst.ptrw();
	const uint8_t *ip = src.ptr();
	const int64_t n = dst.size() / 3;
	bool left = false;
	for (int64_t i = 0; i < n; i++) {
		const int64_t idx = i * 3;
		const float w = ortho_nodata_weight(cp[idx], cp[idx + 1], cp[idx + 2]);
		if (w <= 0.0f) continue;
		for (int c = 0; c < 3; c++) {
			cp[idx + c] = (uint8_t)(cp[idx + c] * (1.0f - w) + ip[idx + c] * w + 0.5f);
		}
		if (ortho_nodata_weight(cp[idx], cp[idx + 1], cp[idx + 2]) > 0.5f) left = true;
	}
	return left;
}

// Fill the remaining (near-)no-data pixels of an RGB8 image from its own
// valid pixels (nearest valid colour, two-pass propagation, O(n)). This is
// what happens beyond the LAST source: Swiss tiles reaching into Austria /
// Liechtenstein / Lake Constance kept SWISSIMAGE's white or black there and
// drew white squares along those borders. Returns the number of valid
// (real) pixels — 0 means the tile lies entirely outside every source.
static int64_t ortho_fill_nodata(PackedByteArray &rgb, int w, int h)
{
	const int64_t n = (int64_t)w * h;
	if (n <= 0 || rgb.size() < n * 3) return 0;
	uint8_t *p = rgb.ptrw();
	std::vector<int32_t> src(n, -1);
	std::vector<int32_t> dist(n, 0x3fffffff);
	int64_t valid = 0;
	for (int64_t i = 0; i < n; i++) {
		if (ortho_nodata_weight(p[i * 3], p[i * 3 + 1], p[i * 3 + 2]) <= 0.5f) {
			src[i] = (int32_t)i;
			dist[i] = 0;
			valid++;
		}
	}
	if (valid == 0 || valid == n) return valid;
	auto relax = [&](int64_t i, int64_t j) {  // j: neighbour of i
		if (src[j] >= 0 && dist[j] + 1 < dist[i]) { dist[i] = dist[j] + 1; src[i] = src[j]; }
	};
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			const int64_t i = (int64_t)y * w + x;
			if (dist[i] == 0) continue;
			if (x > 0) relax(i, i - 1);
			if (y > 0) relax(i, i - w);
		}
	}
	for (int y = h - 1; y >= 0; y--) {
		for (int x = w - 1; x >= 0; x--) {
			const int64_t i = (int64_t)y * w + x;
			if (dist[i] == 0) continue;
			if (x + 1 < w) relax(i, i + 1);
			if (y + 1 < h) relax(i, i + w);
		}
	}
	// Mean colour of the real pixels: the nearest-pixel fill fades to it
	// with distance (over ~1/32 of the tile), otherwise the two-pass fill
	// draws long axis-aligned streaks across large no-data areas.
	uint64_t acc[3] = { 0, 0, 0 };
	for (int64_t i = 0; i < n; i++) {
		if (dist[i] != 0) continue;
		acc[0] += p[i * 3]; acc[1] += p[i * 3 + 1]; acc[2] += p[i * 3 + 2];
	}
	const float mean[3] = { (float)acc[0] / valid, (float)acc[1] / valid, (float)acc[2] / valid };
	const float fade_px = (float)MAX(2, MIN(w, h) / 32);
	for (int64_t i = 0; i < n; i++) {
		if (dist[i] == 0 || src[i] < 0) continue;
		const int64_t s = (int64_t)src[i] * 3;
		const float t = MIN(1.0f, (float)dist[i] / fade_px);
		for (int c = 0; c < 3; c++) {
			p[i * 3 + c] = (uint8_t)(p[s + c] * (1.0f - t) + mean[c] * t + 0.5f);
		}
	}
	return valid;
}

// --- Orthophoto mip path helper ---

int S3DTileManager::ortho_mip_for_lod(int lod)
{
	// On-disk mip level chosen for a given tile LOD:
	//   LOD ≤ 6 → mip0 (1024 px on disk). For LOD 5–6 the in-memory image
	//             is downsampled to 256 px after decoding (see process_load_results)
	//             so GPU memory stays bounded while mid-distance tiles still
	//             look sharp (4 m/px instead of 16 m/px).
	//   LOD 7+  → mip2 (64 px). Far ring; sub-pixel detail is pointless.
	if (lod >= 7) return 2;
	return 0;
}

// In-memory texture size to apply for a given tile LOD. Equals the on-disk
// pixel count for mip0/mip2, but for the LOD 5–6 mid range we deliberately
// downsample the mip0 source to keep GPU/RAM usage manageable.
static int ortho_texture_pixels_for_lod(int lod)
{
	if (lod >= 7) return 64;   // mip2 — used as-is
	if (lod >= 5) return 256;  // mip0 downsampled to 4 m/px
	return 1024;               // mip0 — full 1 m/px
}

std::vector<std::string> S3DTileManager::ortho_paths_for_tile(int ei, int ni, int lod) const
{
	std::vector<std::string> out;
	if (orthophoto_paths.is_empty()) return out;

	int lv95_e = ei * tile_size;
	int lv95_n = ni * tile_size;
	std::string fname = "/ortho_" + std::to_string(lv95_e) + "_" + std::to_string(lv95_n) + ".jpg";
	int mip = ortho_mip_for_lod(lod);
	std::string suffix = (mip == 0) ? fname : ("/mip" + std::to_string(mip) + fname);

	out.reserve(orthophoto_paths.size());
	for (int i = 0; i < orthophoto_paths.size(); i++) {
		String p = orthophoto_paths[i];
		if (p.is_empty()) continue;
		out.push_back(std::string(p.utf8().get_data()) + suffix);
	}
	return out;
}

// Land-cover ground texture of a tile without an orthophoto, pixels per side
// (8 m/px over a 1024 m square): a wood's edge is sharp enough under the tree
// cards and the texture costs 48 KB. Far chunks paint 16 px per sub-tile like
// the mip3 orthophotos.
static constexpr int LANDCOVER_TILE_PX = 128;
static constexpr int LANDCOVER_CHUNK_PX = 16;

std::vector<std::string> S3DTileManager::landcover_paths_for_tile(int ei, int ni) const
{
	std::vector<std::string> out;
	if (landcover_paths.is_empty()) return out;
	const std::string fname = "/forest_" + std::to_string(ei * tile_size) + "_" + std::to_string(ni * tile_size) + ".bin";
	out.reserve(landcover_paths.size());
	for (int i = 0; i < landcover_paths.size(); i++) {
		String p = landcover_paths[i];
		if (p.is_empty()) continue;
		out.push_back(std::string(p.utf8().get_data()) + fname);
	}
	return out;
}

// --- Per-path tile resolution (manifest.json "pixels_per_tile") ---
//
// Every terrain build pipeline writes <terrain>/manifest.json next to the
// raw tiles. Data-light regions bake far fewer pixels per 1024 m cell than
// CH/BW (Norway: 34 px at ~31 m/px vs 512/1024 px), so a single node-wide
// tile_px cannot serve all of them — the worker's hard file-size check
// would reject every tile from the other-resolution paths. The manifest is
// the source of truth; its absence means "same as the node value", which is
// exactly how legacy regions (no pixels_per_tile field) keep working.

// ── Cross-resolution terrain merge ─────────────────────────────────────────
//
// Two terrain sources can cover the same tile at different pixels_per_tile
// (a 1025 px CH/BW detail pack over a 34 px SRTM base pack, design §11 D13).
// Both lattices span the full tile in the same orientation (col 0 = east
// edge, row 0 = south edge), so a pixel (c, r) of one lattice sits at
// (c·(n−1)/(m−1), r·(n−1)/(m−1)) on the other. The finer source is
// authoritative wherever it has data; its no-data (0.0) samples are filled
// from the coarser source, bilinearly resampled from non-zero neighbours
// only. Without this the diagonal no-data wedge along the BW pack's eastern
// edge (its UTM32 source cells projected into LV95) stayed at 0 m next to
// the 30 m base terrain — a dark cliff at the pack boundary.
static float elev_sample_nonzero(const float *src, int src_px, double u, double v)
{
	int c0 = (int)std::floor(u), r0 = (int)std::floor(v);
	int c1 = std::min(c0 + 1, src_px - 1), r1 = std::min(r0 + 1, src_px - 1);
	c0 = std::max(0, std::min(c0, src_px - 1));
	r0 = std::max(0, std::min(r0, src_px - 1));
	double fc = std::max(0.0, std::min(1.0, u - c0));
	double fr = std::max(0.0, std::min(1.0, v - r0));
	const float s[4] = { src[(size_t)r0 * src_px + c0], src[(size_t)r0 * src_px + c1],
		src[(size_t)r1 * src_px + c0], src[(size_t)r1 * src_px + c1] };
	const double w[4] = { (1 - fc) * (1 - fr), fc * (1 - fr), (1 - fc) * fr, fc * fr };
	double num = 0.0, den = 0.0;
	for (int i = 0; i < 4; i++) {
		if (s[i] != 0.0f) { num += s[i] * w[i]; den += w[i]; }
	}
	return den > 0.0 ? (float)(num / den) : 0.0f;
}

// Fill the zero samples of `dst` (dst_px² floats) from `src` (src_px² floats)
// of a different resolution. When `src` is the finer source it becomes the
// new `dst` (its zeros filled from the old, coarser data); dst_px follows.
static void elev_fill_zeros_resampled(std::vector<uint8_t> &dst, int &dst_px,
	std::vector<uint8_t> &src, int src_px)
{
	if (dst_px < 2 || src_px < 2 || dst_px == src_px) return;
	if (src_px > dst_px) {
		std::swap(dst, src);
		std::swap(dst_px, src_px);
	}
	float *d = reinterpret_cast<float *>(dst.data());
	const float *sp = reinterpret_cast<const float *>(src.data());
	const double k = (double)(src_px - 1) / (double)(dst_px - 1);
	for (int r = 0; r < dst_px; r++) {
		for (int c = 0; c < dst_px; c++) {
			float &v = d[(size_t)r * dst_px + c];
			if (v != 0.0f) continue;
			v = elev_sample_nonzero(sp, src_px, c * k, r * k);
		}
	}
}

// The bake pipeline writes a full per-tile list into terrain/manifest.json
// (the Fennoscandia manifests are 140-310 MB). The runtime only needs a few
// scalar fields, which sit at the head of the file, so read the first 64 KB
// and pick them out textually instead of JSON-parsing the whole file (that
// cost 11.7 s per set_data_paths call — at startup and on every settings
// save). A small manifest whose head lacks the key is still fully parsed.
static const int64_t MANIFEST_HEAD_BYTES = 65536;
static const int64_t MANIFEST_FULL_PARSE_MAX = 4 * 1024 * 1024;

static String manifest_head(const std::string &p_dir, int64_t &file_size)
{
	String mpath = String(std::string(p_dir).c_str()) + "/manifest.json";
	Ref<FileAccess> f = FileAccess::open(mpath, FileAccess::READ);
	file_size = 0;
	if (f.is_null()) {
		return String();
	}
	file_size = f->get_length();
	PackedByteArray bytes = f->get_buffer(MIN(file_size, MANIFEST_HEAD_BYTES));
	return String::utf8((const char *)bytes.ptr(), (int)bytes.size());
}

// Value text following "<key>": in a JSON head (up to ',' or '}'), or empty.
static String manifest_head_value(const String &head, const String &key)
{
	int k = head.find(String("\"") + key + "\"");
	if (k < 0) {
		return String();
	}
	int c = head.find(":", k);
	if (c < 0) {
		return String();
	}
	int e = c + 1;
	while (e < head.length() && (head[e] == ' ' || head[e] == '\t' || head[e] == '\n' || head[e] == '\r')) e++;
	int end = e;
	while (end < head.length() && head[end] != ',' && head[end] != '}' && head[end] != '\n') end++;
	return head.substr(e, end - e).strip_edges().trim_prefix("\"").trim_suffix("\"");
}

static Dictionary manifest_full_parse(const std::string &p_dir)
{
	String mpath = String(std::string(p_dir).c_str()) + "/manifest.json";
	Ref<FileAccess> f = FileAccess::open(mpath, FileAccess::READ);
	if (f.is_null()) {
		return Dictionary();
	}
	Variant v = JSON::parse_string(f->get_as_text());
	return v.get_type() == Variant::DICTIONARY ? Dictionary(v) : Dictionary();
}

int S3DTileManager::manifest_pixels_per_tile(const std::string &p_dir)
{
	int64_t size = 0;
	String head = manifest_head(p_dir, size);
	if (head.is_empty()) {
		return 0;
	}
	String v = manifest_head_value(head, "pixels_per_tile");
	if (!v.is_empty()) {
		int px = v.to_int();
		return px > 0 ? px : 0;
	}
	if (size <= MANIFEST_FULL_PARSE_MAX) {
		Dictionary d = manifest_full_parse(p_dir);
		if (d.has("pixels_per_tile")) {
			int px = int(d["pixels_per_tile"]);
			return px > 0 ? px : 0;
		}
	}
	return 0;
}

// EPSG code of a terrain manifest's "crs" field ("EPSG:2056"), 0 when absent.
int S3DTileManager::manifest_crs_epsg(const std::string &p_dir)
{
	int64_t size = 0;
	String head = manifest_head(p_dir, size);
	if (head.is_empty()) {
		return 0;
	}
	String crs = manifest_head_value(head, "crs");
	if (crs.is_empty() && size <= MANIFEST_FULL_PARSE_MAX) {
		Dictionary d = manifest_full_parse(p_dir);
		if (d.has("crs")) {
			crs = String(d["crs"]);
		}
	}
	crs = crs.to_upper().strip_edges();
	if (crs.is_empty()) {
		return 0;
	}
	if (crs.begins_with("EPSG:")) {
		return crs.substr(5).to_int();
	}
	return crs.to_int();
}

int S3DTileManager::tile_px_for_path(const std::string &p_path) const
{
	auto it = path_tile_px.find(p_path);
	if (it != path_tile_px.end() && it->second > 0) {
		return it->second;
	}
	return effective_tile_px();
}

void S3DTileManager::rebuild_path_tile_px()
{
	path_tile_px.clear();
	for (int i = 0; i < data_paths.size(); i++) {
		String p = data_paths[i];
		if (p.is_empty()) continue;
		std::string key(p.utf8().get_data());
		int px = manifest_pixels_per_tile(key);
		if (px > 0) {
			path_tile_px[key] = px;
			UtilityFunctions::print("S3DTileManager: per-path tile_px=", px, " for ", p);
		}
	}
}

// --- Constructor / Destructor ---

S3DTileManager::S3DTileManager()
{
	rebuild_lod_rings();

	shared_material.instantiate();
	shared_material->set_albedo(Color(0.45, 0.55, 0.35));
	shared_material->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
}

S3DTileManager::~S3DTileManager()
{
	stop_worker();
	tiles.clear();
	chunks.clear();
}

// --- LOD ring configuration ---

void S3DTileManager::rebuild_lod_rings()
{
	lod_rings.clear();
	// The tile(s) directly under and immediately around the camera render at
	// LOD0 (full pixel resolution) so the visible mesh is IDENTICAL to the
	// data the S3DElevationDB samples (full-res bilinear). The aircraft is
	// placed on, and baked roads/water sit at, the elevation DB / full-res
	// terrain height, which node_3d.gd also feeds to JSBSim every frame as the
	// ground plane. A coarse near mesh would diverge from that surface and make
	// the aircraft "sink" and roads "float". Beyond the LOD0 ring the coarse
	// tiles use block-averaged heights (see S3DTile::generate_mesh) so they
	// still track the true surface closely; LOD grades out gently — LOD≤2
	// (≤8 m cells) out to ~4 km and LOD3 (16 m) out to ~8 km, which is the zone
	// where the baked road/water overlays are drawn. Only the immediate ring
	// pays the full LOD0 vertex cost. Tiles below LOD_DISCARD_THRESHOLD (4)
	// keep their heightmap in the elevation DB.
	if (load_radius <= 4) {
		lod_rings.push_back({1, 0});
		lod_rings.push_back({2, 1});
		lod_rings.push_back({load_radius, 2});
	} else if (load_radius <= 15) {
		lod_rings.push_back({1, 0});
		lod_rings.push_back({2, 1});
		lod_rings.push_back({4, 2});
		lod_rings.push_back({load_radius, 3});
	} else if (load_radius <= 50) {
		lod_rings.push_back({1, 0});
		lod_rings.push_back({2, 1});
		lod_rings.push_back({4, 2});
		lod_rings.push_back({8, 3});
		lod_rings.push_back({12, 4});
		lod_rings.push_back({load_radius, 6});
	} else {
		lod_rings.push_back({1, 0});
		lod_rings.push_back({2, 1});
		lod_rings.push_back({4, 2});
		lod_rings.push_back({8, 3});
		lod_rings.push_back({16, 4});
		lod_rings.push_back({30, 6});
		lod_rings.push_back({load_radius, 8});
	}
}

int S3DTileManager::lod_for_distance(int dist) const
{
	for (const auto &ring : lod_rings) {
		if (dist <= ring.radius) {
			return ring.lod_level;
		}
	}
	return lod_rings.back().lod_level;
}

int S3DTileManager::chunk_res_for_distance(int dist, int current_res) const
{
	// Hysteresis: a chunk only drops to a coarser resolution once the
	// camera is CHUNK_RES_HYSTERESIS_TILES past the threshold, so flying
	// along a threshold does not rebuild the same chunks back and forth.
	int near_lim = CHUNK_NEAR_TILES + (current_res == CHUNK_RES_NEAR ? CHUNK_RES_HYSTERESIS_TILES : 0);
	int mid_lim = CHUNK_MID_TILES + (current_res == CHUNK_RES_MID ? CHUNK_RES_HYSTERESIS_TILES : 0);
	if (dist <= near_lim) return CHUNK_RES_NEAR;
	if (dist <= mid_lim) return CHUNK_RES_MID;
	return CHUNK_COMPOSITE_RES;
}

// --- Background worker thread pool ---

void S3DTileManager::start_worker()
{
	if (worker_running.load()) return;
	worker_running.store(true);
	for (int i = 0; i < NUM_WORKERS; i++) {
		worker_threads.emplace_back(&S3DTileManager::worker_func, this);
	}
}

void S3DTileManager::stop_worker()
{
	if (!worker_running.load()) return;
	worker_running.store(false);
	work_cv.notify_all();
	for (auto &t : worker_threads) {
		if (t.joinable()) t.join();
	}
	worker_threads.clear();
}

void S3DTileManager::worker_func()
{
	while (worker_running.load()) {
		LoadRequest req;
		{
			std::unique_lock<std::mutex> lock(work_mutex);
			work_cv.wait(lock, [this] {
				return !work_queue.empty() || !worker_running.load();
			});
			if (!worker_running.load()) break;
			if (work_queue.empty()) continue;
			req = std::move(work_queue.front());
			work_queue.pop_front();
		}

		LoadResult result;
		result.ei = req.ei;
		result.ni = req.ni;
		result.key = req.key;
		result.is_chunk = req.is_chunk;
		result.success = false;

		if (req.is_chunk) {
			// --- Chunk: read multiple tile files, composite into small heightmap ---
			int cres = req.composite_res;
			int csize = req.chunk_tile_count;
			int base_ei = req.chunk_grid_ei * csize;
			int base_ni = req.chunk_grid_ni * csize;
			int ts_m = req.tile_size; // meters per source tile axis
			int ts_px = req.tile_px > 0 ? req.tile_px : req.tile_size; // pixels per source tile axis

			result.composite_res = cres;
			result.tile_size = csize * ts_m;
			result.desired_lod = 0;

			std::vector<float> composite(cres * cres, NAN);
			bool any_data = false;

			// Cache tile data to avoid re-reading the same file. The px
			// field records the resolution the bytes were loaded at — a
			// chunk may span sources with different pixels_per_tile, and
			// the sampling math below must use each entry's own grid.
			struct CachedTile {
				std::vector<uint8_t> data;
				int px = 0;
			};
			std::unordered_map<uint64_t, CachedTile> tile_cache;

			for (int cy = 0; cy < cres; cy++) {
				for (int cx = 0; cx < cres; cx++) {
					// Composite pixel (0,0) = SE corner of chunk.
					// cx goes E→W, cy goes S→N (matching provpilot convention).
					double frac_x = (double)cx / (cres - 1); // 0=east edge, 1=west edge
					double frac_y = (double)cy / (cres - 1); // 0=south edge, 1=north edge

					double lv95_e = (double)(base_ei + csize) * ts_m - frac_x * csize * ts_m;
					double lv95_n = (double)base_ni * ts_m + frac_y * csize * ts_m;

					int tei = (int)std::floor(lv95_e / ts_m);
					int tni = (int)std::floor(lv95_n / ts_m);

					// Clamp to chunk bounds.
					if (tei < base_ei) tei = base_ei;
					if (tei >= base_ei + csize) tei = base_ei + csize - 1;
					if (tni < base_ni) tni = base_ni;
					if (tni >= base_ni + csize) tni = base_ni + csize - 1;

					uint64_t tkey = tile_key(tei, tni);
					auto cache_it = tile_cache.find(tkey);
					if (cache_it == tile_cache.end()) {
						int lv95_east = tei * ts_m;
						int lv95_north = tni * ts_m;
						std::string tname = "/tile_"
							+ std::to_string(lv95_east) + "_"
							+ std::to_string(lv95_north) + ".raw";
						std::vector<uint8_t> data;
						std::vector<uint8_t> scratch;
						int data_px = 0; // Resolution of the winning source.
						for (size_t bi = 0; bi < req.base_paths.size(); bi++) {
							// Each base path may store tiles at a different
							// pixels_per_tile (manifest override); fall back
							// to the node-wide value when undeclared.
							int px_b = (bi < req.base_path_px.size() && req.base_path_px[bi] > 0)
								? req.base_path_px[bi]
								: ts_px;
							size_t expected_b = (size_t)px_b * px_b * 4;
							std::ifstream file(req.base_paths[bi] + tname, std::ios::binary);
							if (!file.is_open()) continue;
							scratch.resize(expected_b);
							file.read(reinterpret_cast<char *>(scratch.data()), expected_b);
							if ((size_t)file.gcount() != expected_b) {
								scratch.clear();
								continue;
							}
							if (data.empty()) {
								data = std::move(scratch);
								data_px = px_b;
								scratch.clear();
							} else if (scratch.size() == data.size()) {
								// Merge: fill zero samples from this source.
								float *dst = reinterpret_cast<float *>(data.data());
								const float *src = reinterpret_cast<const float *>(scratch.data());
								size_t n = data.size() / 4;
								for (size_t i = 0; i < n; ++i) {
									if (dst[i] == 0.0f && src[i] != 0.0f) {
										dst[i] = src[i];
									}
								}
							} else {
								// Different resolutions: the finer source
								// wins, the coarser one fills its no-data.
								elev_fill_zeros_resampled(data, data_px, scratch, px_b);
							}
						}
						// Gap-fill the freshly loaded tile before sampling: the
						// raw data from the BW / SwissTopo pipeline can contain
						// rows/columns of zero (water, nodata) that are wider
						// than the 33×33 composite's 3×3 neighbourhood fill can
						// bridge. Use the same 4-pass row/col sweep the regular
						// tile path uses, so every sampled pixel comes back
						// with a plausible elevation.
						if (!data.empty()) {
							float *d = reinterpret_cast<float *>(data.data());
							int sz = data_px;
							for (int r = 0; r < sz; r++) {
								float last = 0.0f;
								for (int c = 0; c < sz; c++) {
									float &v = d[r * sz + c];
									if (v != 0.0f) last = v;
									else if (last != 0.0f) v = last;
								}
								last = 0.0f;
								for (int c = sz - 1; c >= 0; c--) {
									float &v = d[r * sz + c];
									if (v != 0.0f) last = v;
									else if (last != 0.0f) v = last;
								}
							}
							for (int c = 0; c < sz; c++) {
								float last = 0.0f;
								for (int r = 0; r < sz; r++) {
									float &v = d[r * sz + c];
									if (v != 0.0f) last = v;
									else if (last != 0.0f) v = last;
								}
								last = 0.0f;
								for (int r = sz - 1; r >= 0; r--) {
									float &v = d[r * sz + c];
									if (v != 0.0f) last = v;
									else if (last != 0.0f) v = last;
								}
							}
						}
						tile_cache[tkey] = CachedTile{std::move(data), data_px};
						cache_it = tile_cache.find(tkey);
					}

					if (cache_it->second.data.empty()) continue;

					// Block-average sample from the raw tile. Each composite
					// cell covers (ts / (cres-1)) source pixels per axis, so
					// we average a small kx×ky grid within that block rather
					// than picking a single pixel. This is robust against
					// isolated zero / nodata pixels that slip past the
					// per-tile gap-fill above, and dramatically reduces the
					// chance of a whole chunk being classified as empty.
					const float *tile_f = reinterpret_cast<const float *>(cache_it->second.data.data());
					// Per-tile pixel grid: the cached bytes came from a source
					// whose manifest may declare a different pixels_per_tile
					// than the node-wide value.
					int tpx_t = cache_it->second.px > 0 ? cache_it->second.px : ts_px;
					double local_e = lv95_e - (double)tei * ts_m;
					double local_n = lv95_n - (double)tni * ts_m;

					int pcol_c = (int)std::round((ts_m - local_e) / ts_m * (tpx_t - 1));
					int prow_c = (int)std::round(local_n / ts_m * (tpx_t - 1));
					pcol_c = std::max(0, std::min(tpx_t - 1, pcol_c));
					prow_c = std::max(0, std::min(tpx_t - 1, prow_c));

					// Supersampling window: ±K pixels around the nominal
					// sample, clamped to the tile. K ≈ half a composite step
					// (tpx_t / (cres-1) / 2), capped to keep cost bounded.
					int half = std::min(8, (int)(tpx_t / (cres - 1) / 2));

					float sum = 0.0f;
					int cnt = 0;
					for (int dy = -half; dy <= half; dy++) {
						int pr = prow_c + dy;
						if (pr < 0 || pr >= tpx_t) continue;
						for (int dx = -half; dx <= half; dx++) {
							int pc = pcol_c + dx;
							if (pc < 0 || pc >= tpx_t) continue;
							float v = tile_f[(size_t)pr * tpx_t + pc];
							if (v != 0.0f) { sum += v; cnt++; }
						}
					}
					if (cnt > 0) {
						composite[cy * cres + cx] = sum / (float)cnt;
						any_data = true;
					}
				}
			}

			// Fill NaN gaps by propagating nearest valid elevation.
			if (any_data) {
				for (int pass = 0; pass < cres; pass++) {
					bool changed = false;
					std::vector<float> prev = composite;
					for (int i = 0; i < cres * cres; i++) {
						if (!std::isnan(prev[i])) continue;
						int cy = i / cres, cx = i % cres;
						float sum = 0; int cnt = 0;
						for (int dy = -1; dy <= 1; dy++) {
							for (int dx = -1; dx <= 1; dx++) {
								int ny = cy + dy, nx = cx + dx;
								if (ny >= 0 && ny < cres && nx >= 0 && nx < cres) {
									float v = prev[ny * cres + nx];
									if (!std::isnan(v)) { sum += v; cnt++; }
								}
							}
						}
						if (cnt > 0) { composite[i] = sum / cnt; changed = true; }
					}
					if (!changed) break;
				}
				// Any remaining NaN → 0 (shouldn't happen if chunk has data).
				for (auto &v : composite) {
					if (std::isnan(v)) v = 0.0f;
				}
			}

			if (any_data) {
				result.raw_bytes.resize(cres * cres * 4);
				memcpy(result.raw_bytes.data(), composite.data(), cres * cres * 4);
				result.success = true;
			}

			// --- Read chunk ortho JPEG tiles (mip3, 16x16 each) ---
			if (any_data && (!req.ortho_mip_dirs.empty() || !req.landcover_dirs.empty())) {
				result.chunk_tile_count = csize;
				for (int tdy = 0; tdy < csize; tdy++) {
					for (int tdx = 0; tdx < csize; tdx++) {
						int tei = base_ei + tdx;
						int tni = base_ni + tdy;
						int lv95_e = tei * ts_m;
						int lv95_n = tni * ts_m;
						std::string fname = "/ortho_"
							+ std::to_string(lv95_e) + "_" + std::to_string(lv95_n) + ".jpg";

						// Try each ortho root in priority order. With
						// skip_white_pixels and 2+ roots the mip3 tiles are
						// composited like the full-res ones (no-data pixels
						// of a higher-priority source filled from the next);
						// first-hit-wins otherwise. Without this the far
						// chunks showed SWISSIMAGE's white/black margins as
						// a wide stripe along the CH/DE border.
						const bool chunk_composite = skip_white_pixels && req.ortho_mip_dirs.size() >= 2;
						std::vector<uint8_t> jbuf;
						Ref<Image> comp;
						bool nodata_left = true;
						for (const auto &dir : req.ortho_mip_dirs) {
							if (chunk_composite && comp.is_valid() && !nodata_left) break;
							std::string opath = dir + fname;
							std::ifstream jpf(opath, std::ios::binary | std::ios::ate);
							if (!jpf.is_open()) continue;
							size_t fsize = (size_t)jpf.tellg();
							if (fsize == 0 || fsize > 256 * 1024) continue;
							jpf.seekg(0, std::ios::beg);
							jbuf.resize(fsize);
							jpf.read(reinterpret_cast<char *>(jbuf.data()), fsize);
							if ((size_t)jpf.gcount() != fsize) { jbuf.clear(); continue; }
							if (!chunk_composite) break;
							PackedByteArray jpkg;
							jpkg.resize((int)jbuf.size());
							memcpy(jpkg.ptrw(), jbuf.data(), jbuf.size());
							Ref<Image> img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
							if (img->load_jpg_from_buffer(jpkg) != OK || img->get_width() < 2) continue;
							if (img->get_format() != Image::FORMAT_RGB8) img->convert(Image::FORMAT_RGB8);
							if (comp.is_null()) {
								comp = img;
								PackedByteArray d = comp->get_data();
								nodata_left = false;
								const uint8_t *p = d.ptr();
								for (int64_t i = 0; i + 2 < d.size(); i += 3) {
									if (ortho_is_nodata(p[i], p[i + 1], p[i + 2])) { nodata_left = true; break; }
								}
							} else if (img->get_width() == comp->get_width() && img->get_height() == comp->get_height()) {
								PackedByteArray cdata = comp->get_data();
								nodata_left = ortho_merge_nodata(cdata, img->get_data());
								comp = Image::create_from_data(comp->get_width(), comp->get_height(), false, Image::FORMAT_RGB8, cdata);
							}
						}
						int key = tdx * csize + tdy;
						if (chunk_composite && comp.is_valid()) {
							LoadResult::RawRgb raw;
							raw.w = comp->get_width();
							PackedByteArray d = comp->get_data();
							raw.rgb.assign(d.ptr(), d.ptr() + d.size());
							result.chunk_ortho_rgb[key] = std::move(raw);
							continue;
						}
						if (jbuf.empty()) {
							// No orthophoto for this sub-tile: paint its
							// land cover (forest polygons, or the plain
							// base colour) wherever it has terrain, so the
							// no-data fill below does not smear a neighbour's
							// pixels across it (design §14.6).
							if (req.landcover_dirs.empty()) continue;
							auto cache_it = tile_cache.find(tile_key(tei, tni));
							if (cache_it == tile_cache.end() || cache_it->second.data.empty()) continue;
							std::vector<std::string> cands;
							cands.reserve(req.landcover_dirs.size());
							const std::string fbin = "/forest_" + std::to_string(lv95_e) + "_" + std::to_string(lv95_n) + ".bin";
							for (const auto &dir : req.landcover_dirs) cands.push_back(dir + fbin);
							LoadResult::RawRgb raw;
							raw.w = LANDCOVER_CHUNK_PX;
							if (!s3d_landcover_raster(cands, LANDCOVER_CHUNK_PX, (float)ts_m, raw.rgb)) {
								s3d_fill_landcover_base(LANDCOVER_CHUNK_PX, raw.rgb);
							}
							result.chunk_ortho_rgb[key] = std::move(raw);
							result.landcover_painted = true;
							continue;
						}
						result.chunk_ortho_jpegs[key] = std::move(jbuf);
					}
				}
			}
		} else {
			// --- Regular tile: read single file, trying each candidate path ---
			result.tile_size = req.tile_size;
			result.desired_lod = req.desired_lod;

			// Read all sources and merge per-pixel, with LATER sources
			// overriding earlier ones at any pixel where the later source
			// is non-zero. This lets the user encode authoritative priority
			// via `data_paths` order: e.g. ["Switzerland", "BW"] means BW
			// wins for German pixels where it has data, falling back to CH
			// elsewhere — important because CH ALTI3D extends beyond the
			// border with extrapolated values that disagree with BW DGM by
			// up to 40 m. A simple "first source wholesale" or "biggest
			// coverage wins" rule fails on tiles where CH has 100 %
			// coverage but only the DE corner is real ground truth.
			//
			// Each candidate path may store its tiles at a different pixel
			// resolution (manifest.json pixels_per_tile), so the expected
			// file size and the merge are per-candidate. Sources of a
			// different resolution are merged by elev_fill_zeros_resampled:
			// the finer source is authoritative, the coarser one fills its
			// no-data samples (a 34 px base pack under the 1025 px CH/BW
			// detail packs, design §11 D13).
			std::vector<uint8_t> scratch;
			for (size_t i = 0; i < req.paths.size(); i++) {
				int px_i = (i < req.path_px.size() && req.path_px[i] > 0)
					? req.path_px[i]
					: (req.tile_px > 0 ? req.tile_px : req.tile_size);
				size_t expected_size = (size_t)px_i * px_i * 4;
				std::ifstream file(req.paths[i], std::ios::binary);
				if (!file.is_open()) {
					continue;
				}
				scratch.resize(expected_size);
				file.read(reinterpret_cast<char *>(scratch.data()), expected_size);
				if ((size_t)file.gcount() != expected_size) {
					continue;
				}

				if (!result.success) {
					result.raw_bytes = std::move(scratch);
					scratch.clear();
					result.tile_px = px_i; // Winning source's resolution.
					result.success = true;
				} else if (scratch.size() == result.raw_bytes.size()) {
					float *dst = reinterpret_cast<float *>(result.raw_bytes.data());
					const float *src = reinterpret_cast<const float *>(scratch.data());
					size_t sample_count = scratch.size() / 4;
					for (size_t k = 0; k < sample_count; ++k) {
						if (src[k] != 0.0f) {
							dst[k] = src[k];
						}
					}
				} else {
					elev_fill_zeros_resampled(result.raw_bytes, result.tile_px, scratch, px_i);
				}
			}

			// Fill remaining zero-elevation gaps (nodata from BW DGM etc.)
			// by inverse-distance interpolation from the four nearest
			// valid samples along the row and column. Previously we did a
			// 4-pass smear that simply copied the last valid sample into
			// each gap; for slanted wedges of nodata (caused by missing
			// UTM32 source cells projecting diagonally into LV95) that
			// stretches a single edge sample across hundreds of columns,
			// producing the long horizontal streaks visible in BW.
			if (result.success) {
				float *d = reinterpret_cast<float *>(result.raw_bytes.data());
				int sz = result.tile_px;
				size_t total = (size_t)sz * sz;
				std::vector<float> Lv(total, 0.0f), Rv(total, 0.0f);
				std::vector<float> Uv(total, 0.0f), Dv(total, 0.0f);
				std::vector<int> Ld(total, 0), Rd(total, 0);
				std::vector<int> Ud(total, 0), Dd(total, 0);
				// Horizontal scans (per row).
				for (int r = 0; r < sz; r++) {
					float v = 0.0f; int dist = 0; bool have = false;
					for (int c = 0; c < sz; c++) {
						size_t i = (size_t)r * sz + c;
						if (d[i] != 0.0f) { v = d[i]; dist = 0; have = true; }
						else if (have) { dist++; }
						if (have && d[i] == 0.0f) { Lv[i] = v; Ld[i] = dist; }
					}
					v = 0.0f; dist = 0; have = false;
					for (int c = sz - 1; c >= 0; c--) {
						size_t i = (size_t)r * sz + c;
						if (d[i] != 0.0f) { v = d[i]; dist = 0; have = true; }
						else if (have) { dist++; }
						if (have && d[i] == 0.0f) { Rv[i] = v; Rd[i] = dist; }
					}
				}
				// Vertical scans (per column).
				for (int c = 0; c < sz; c++) {
					float v = 0.0f; int dist = 0; bool have = false;
					for (int r = 0; r < sz; r++) {
						size_t i = (size_t)r * sz + c;
						if (d[i] != 0.0f) { v = d[i]; dist = 0; have = true; }
						else if (have) { dist++; }
						if (have && d[i] == 0.0f) { Uv[i] = v; Ud[i] = dist; }
					}
					v = 0.0f; dist = 0; have = false;
					for (int r = sz - 1; r >= 0; r--) {
						size_t i = (size_t)r * sz + c;
						if (d[i] != 0.0f) { v = d[i]; dist = 0; have = true; }
						else if (have) { dist++; }
						if (have && d[i] == 0.0f) { Dv[i] = v; Dd[i] = dist; }
					}
				}
				// Inverse-distance blend of whichever directions found a
				// valid neighbour. A 1-pixel-away sample dominates a
				// 100-pixel-away sample, so this approximates linear
				// interpolation across narrow gaps and graceful blending
				// across larger ones.
				for (size_t i = 0; i < total; i++) {
					if (d[i] != 0.0f) continue;
					float num = 0.0f, den = 0.0f;
					if (Lv[i] != 0.0f) { float w = 1.0f / (float)Ld[i]; num += Lv[i] * w; den += w; }
					if (Rv[i] != 0.0f) { float w = 1.0f / (float)Rd[i]; num += Rv[i] * w; den += w; }
					if (Uv[i] != 0.0f) { float w = 1.0f / (float)Ud[i]; num += Uv[i] * w; den += w; }
					if (Dv[i] != 0.0f) { float w = 1.0f / (float)Dd[i]; num += Dv[i] * w; den += w; }
					if (den > 0.0f) d[i] = num / den;
				}
			}

			// Load orthophoto JPEG: try each candidate path, first that
			// opens and reads cleanly wins. Lets CH SWISSIMAGE and DE
			// LGL DOP20 share the same tile grid.
			//
			// When skip_white_pixels is enabled and 2+ candidates exist,
			// every candidate is decoded and pure-white (255,255,255)
			// pixels in higher-priority sources are filled in from
			// lower-priority ones. SWISSIMAGE encodes German territory
			// at the CH/DE border as solid white (up to 43% of a 1024²
			// tile); without this, that white shows through even when
			// BW DOP20 covers the same area.
			if (result.success && !req.ortho_paths.empty()) {
				const bool composite_mode = skip_white_pixels && req.ortho_paths.size() >= 2;
				Ref<Image> composite_img;
				bool any_white_left = true;

				for (const auto &cand : req.ortho_paths) {
					if (composite_mode && composite_img.is_valid() && !any_white_left) break;

					std::ifstream jpf(cand, std::ios::binary | std::ios::ate);
					if (!jpf.is_open()) continue;
					size_t fsize = (size_t)jpf.tellg();
					if (fsize == 0 || fsize >= 16 * 1024 * 1024) continue;
					jpf.seekg(0, std::ios::beg);
					std::vector<uint8_t> jbuf(fsize);
					jpf.read(reinterpret_cast<char *>(jbuf.data()), fsize);
					if ((size_t)jpf.gcount() != fsize) continue;

					if (!composite_mode) {
						// Fast path: original first-hit-wins behaviour.
						result.jpeg_bytes = std::move(jbuf);
						result.ortho_pixels = req.ortho_pixels;
						break;
					}

					// Composite path: decode and merge into composite.
					PackedByteArray jpkg;
					jpkg.resize((int)jbuf.size());
					memcpy(jpkg.ptrw(), jbuf.data(), jbuf.size());
					Ref<Image> img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
					if (img->load_jpg_from_buffer(jpkg) != OK) continue;
					if (img->get_width() < 2) continue;
					if (img->get_format() != Image::FORMAT_RGB8) {
						img->convert(Image::FORMAT_RGB8);
					}

					if (composite_img.is_null()) {
						composite_img = img;
					} else {
						// Same dimensions required to fill pixel-for-pixel.
						if (img->get_width() != composite_img->get_width()
								|| img->get_height() != composite_img->get_height()) {
							continue;
						}
						PackedByteArray cdata = composite_img->get_data();
						PackedByteArray idata = img->get_data();
						ortho_merge_nodata(cdata, idata);
						composite_img = Image::create_from_data(
							composite_img->get_width(), composite_img->get_height(),
							false, Image::FORMAT_RGB8, cdata);
					}

					// Quick scan: any no-data pixels remaining? If not, stop.
					{
						PackedByteArray cdata = composite_img->get_data();
						const uint8_t *cp = cdata.ptr();
						int npx = composite_img->get_width() * composite_img->get_height();
						bool found = false;
						// Sample stride to keep this cheap; full check on small images.
						int step = npx > 4096 ? 16 : 1;
						for (int i = 0; i < npx; i += step) {
							int idx = i * 3;
							if (ortho_is_nodata(cp[idx], cp[idx + 1], cp[idx + 2])) {
								found = true;
								break;
							}
						}
						any_white_left = found;
					}
				}

				// Final pass (skip_white_pixels): whatever no-data is left after
				// the sources are merged — or in a single-source tile — is filled
				// from the tile's own pixels; a tile with (almost) no real pixels
				// lies outside every source and is dropped so the coarse globe
				// shows instead of a white/black square.
				Ref<Image> final_img = composite_img;
				if (skip_white_pixels && final_img.is_null() && !result.jpeg_bytes.empty()) {
					PackedByteArray jpkg;
					jpkg.resize((int)result.jpeg_bytes.size());
					memcpy(jpkg.ptrw(), result.jpeg_bytes.data(), result.jpeg_bytes.size());
					Ref<Image> img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
					if (img->load_jpg_from_buffer(jpkg) == OK && img->get_width() >= 2) {
						if (img->get_format() != Image::FORMAT_RGB8) img->convert(Image::FORMAT_RGB8);
						final_img = img;
					}
				}
				if (skip_white_pixels && final_img.is_valid()) {
					const int w = final_img->get_width(), h = final_img->get_height();
					PackedByteArray d = final_img->get_data();
					const int64_t valid = ortho_fill_nodata(d, w, h);
					if (valid * 10 < (int64_t)w * h) {
						// < 10 % real pixels: outside every source. With
						// land cover the tile is painted from its forest
						// file below; without, it is dropped so the coarse
						// globe shows instead of a white/black square.
						result.jpeg_bytes.clear();
						result.ortho_rgb.clear();
						result.ortho_pixels = -1;
						if (req.landcover_paths.empty()) result.success = false;
					} else {
						result.ortho_rgb.assign(d.ptr(), d.ptr() + d.size());
						result.ortho_rgb_w = w;
						result.ortho_pixels = req.ortho_pixels;
						result.jpeg_bytes.clear();
					}
				} else if (composite_mode && composite_img.is_valid()) {
					PackedByteArray cdata = composite_img->get_data();
					result.ortho_rgb.assign(cdata.ptr(), cdata.ptr() + cdata.size());
					result.ortho_rgb_w = composite_img->get_width();
					result.ortho_pixels = req.ortho_pixels;
				}
			}
			// Land-cover fallback (design §14.6): no orthophoto covers the
			// tile, so its forest polygons are painted into a small ground
			// texture. A square without a forest file keeps the shared
			// material, which is the same colour as the raster's base.
			if (result.success && result.jpeg_bytes.empty() && result.ortho_rgb.empty() && !req.landcover_paths.empty()) {
				std::vector<uint8_t> rgb;
				if (s3d_landcover_raster(req.landcover_paths, LANDCOVER_TILE_PX, (float)req.tile_size, rgb)) {
					result.ortho_rgb = std::move(rgb);
					result.ortho_rgb_w = LANDCOVER_TILE_PX;
					result.ortho_pixels = LANDCOVER_TILE_PX;
				}
			}
		}

		{
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
		}
	}
}

// --- Process completed file loads on main thread ---

static bool heightmap_is_large(const Ref<Image> &img)
{
	if (img.is_null()) return false;
	return (int64_t)img->get_width() * img->get_height() * 4 > 64 * 1024;
}

void S3DTileManager::process_load_results(int &verts_generated)
{
	int chunks_done = 0;
	int ortho_decodes = 0;
	while (true) {
		LoadResult result;
		{
			std::lock_guard<std::mutex> lock(results_mutex);
			if (results_queue.empty()) break;
			result = std::move(results_queue.front());
			results_queue.pop_front();
		}

		if (result.is_chunk) {
			// Throttle chunk processing — JPEG composite is expensive.
			if (chunks_done >= CHUNKS_PER_FRAME) {
				std::lock_guard<std::mutex> lock(results_mutex);
				results_queue.push_front(std::move(result));
				break;
			}
			// --- Process chunk result ---
			auto it = chunks.find(result.key);
			if (it == chunks.end()) continue;

			TileState &state = it->second;
			state.loading = false;

			if (!result.success) {
				state.no_data = true;
				continue;
			}

			int cres = result.composite_res;
			int chunk_verts = cres * cres;
			if (verts_generated + chunk_verts > VERTEX_BUDGET_PER_FRAME && verts_generated > 0) {
				state.loading = true;
				std::lock_guard<std::mutex> lock(results_mutex);
				results_queue.push_front(std::move(result));
				break;
			}

			PackedByteArray bytes;
			bytes.resize(result.raw_bytes.size());
			memcpy(bytes.ptrw(), result.raw_bytes.data(), result.raw_bytes.size());

			Ref<Image> heightmap = Image::create_from_data(
				cres, cres, false, Image::FORMAT_RF, bytes);
			if (heightmap.is_null()) {
				state.no_data = true;
				continue;
			}

			if (!state.node) {
				S3DTile *tile = memnew(S3DTile);
				tile->set_tile_x(result.ei);
				tile->set_tile_z(result.ni);
				// Chunk covers chunk_size * tile_size meters, not just tile_size.
				tile->set_tile_size(tile_size * chunk_size);

				// Composite ortho texture from per-tile mip3 JPEGs.
				bool ortho_applied = false;
				if ((!result.chunk_ortho_jpegs.empty() || !result.chunk_ortho_rgb.empty()) && result.chunk_tile_count > 0) {
					int cs = result.chunk_tile_count;
					int mip_px = 16; // mip3 pixels per tile
					int comp_size = cs * mip_px;
					// Create composite image. Pixel layout matches terrain UV:
					// U=0 at west (high ei), U=1 at east (low ei)
					// V=0 at south (low ni), V=1 at north (high ni)
					Ref<Image> comp_img = Image::create_empty(comp_size, comp_size, false, Image::FORMAT_RGB8);
					bool any_decoded = false;

					// Worker-composited tiles first (raw RGB), then plain JPEGs.
					std::vector<std::pair<int, Ref<Image>>> chunk_imgs;
					for (auto &[tkey, raw] : result.chunk_ortho_rgb) {
						if (raw.w < 2 || raw.rgb.size() < (size_t)raw.w * raw.w * 3) continue;
						PackedByteArray d;
						d.resize((int64_t)raw.rgb.size());
						memcpy(d.ptrw(), raw.rgb.data(), raw.rgb.size());
						chunk_imgs.emplace_back(tkey, Image::create_from_data(raw.w, raw.w, false, Image::FORMAT_RGB8, d));
					}
					for (auto &[tkey, jbuf] : result.chunk_ortho_jpegs) {
						PackedByteArray jpkg;
						jpkg.resize(jbuf.size());
						memcpy(jpkg.ptrw(), jbuf.data(), jbuf.size());
						Ref<Image> tile_img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
						Error err = tile_img->load_jpg_from_buffer(jpkg);
						if (err != OK || tile_img->get_width() < 2) continue;
						chunk_imgs.emplace_back(tkey, tile_img);
					}
					for (auto &[tkey, tile_img] : chunk_imgs) {
						int tdx = tkey / cs; // local x (easting offset)
						int tdy = tkey % cs; // local y (northing offset)

						// Ensure correct format
						if (tile_img->get_format() != Image::FORMAT_RGB8) {
							tile_img->convert(Image::FORMAT_RGB8);
						}

						// Ortho pixel (0,0) = SE corner, cols go E→W, rows go S→N.
						// Terrain UV: u=0 at east edge, v=0 at south edge.
						// Image row 0 = top = UV v=0 = south.
						// tdx=0 is base_ei (westmost), tdx=cs-1 is eastmost.
						// Composite col 0 = east (u=0) → eastmost tile (tdx=cs-1).
						int dst_col = (cs - 1 - tdx) * mip_px;
						// tdy=0 is base_ni (south) → image row 0 (top, v=0).
						int dst_row = tdy * mip_px;

						int tw = tile_img->get_width();
						int th = tile_img->get_height();
						Rect2i src_rect(0, 0, tw < mip_px ? tw : mip_px, th < mip_px ? th : mip_px);
						comp_img->blit_rect(tile_img, src_rect, Vector2i(dst_col, dst_row));
						any_decoded = true;
					}

					bool chunk_outside = false;
					if (any_decoded && skip_white_pixels) {
						// Missing tiles stayed black in the composite; fill them and
						// the no-data margins from the chunk's real pixels.
						PackedByteArray cd = comp_img->get_data();
						const int64_t valid = ortho_fill_nodata(cd, comp_size, comp_size);
						if (valid * 10 < (int64_t)comp_size * comp_size && !result.landcover_painted) {
							chunk_outside = true;
						} else {
							comp_img = Image::create_from_data(comp_size, comp_size, false, Image::FORMAT_RGB8, cd);
						}
					}
					if (chunk_outside) {
						memdelete(tile);
						state.no_data = true;
						continue;
					}
					if (any_decoded) {
						// Mipmaps: the 16 px per square land cover is high
						// contrast and shimmers at grazing angles without them.
						comp_img->generate_mipmaps();
						Ref<ImageTexture> tex = ImageTexture::create_from_image(comp_img);
						tile->set_material(make_chunk_material(result.ei, result.ni, tex));
						ortho_applied = true;
					}
				}
				if (!ortho_applied) {
					tile->set_material(make_chunk_material(result.ei, result.ni, Ref<Texture2D>()));
				}

				int base_ei = result.ei * chunk_size;
				int base_ni = result.ni * chunk_size;
				// Sink chunks slightly below individual tiles (along local up)
				// so where the two overlap the near terrain wins the depth
				// test cleanly. Chunks sample every ~250 m so their surface
				// can otherwise protrude above the detailed tile mesh along
				// ridgelines. In ECEF mode chunks are placed EXACTLY (every
				// vertex projected) — an 8 km affine tile would sag 1.25 m
				// and its seams would mismatch by ~1 m far from the CRS origin.
				add_child(tile);
				state.node = tile;
				bump_coverage();
			}

			{
				// (Re)place on every build: the Y bias follows the composite
				// spacing, and a chunk rebuilt at a finer resolution moves
				// up accordingly.
				const double ts = (double)tile_size;
				const double spacing = (double)chunk_size * ts / (double)(cres - 1);
				const double bias = CHUNK_Y_BIAS * (spacing / CHUNK_Y_BIAS_REF_SPACING_M);
				int base_ei = result.ei * chunk_size;
				int base_ni = result.ni * chunk_size;
				state.node->set_placement(frame,
					(double)(base_ei + chunk_size) * ts, (double)base_ni * ts,
					((double)base_ei + 0.5 * chunk_size) * ts, ((double)base_ni + 0.5 * chunk_size) * ts,
					bias, frame.ecef());
			}

			state.node->set_heightmap(heightmap);
			state.node->set_lod_level(0); // Use every pixel.
			state.node->generate_mesh();
			state.current_lod = 0;
			state.chunk_res = cres;
			state.node->set_heightmap(Ref<Image>()); // Free memory.
			verts_generated += chunk_verts;
			chunks_done++;
			continue;
		}

		// --- Process regular tile result ---
		auto it = tiles.find(result.key);
		if (it == tiles.end()) {
			continue;
		}

		TileState &state = it->second;

		// If this result carries an ortho JPEG that needs decoding +
		// texture upload on the main thread, throttle it. Without this,
		// crossing into a new chunk floods the frame with dozens of
		// 1024² JPEG decodes and the simulation visibly stalls.
		if (!result.jpeg_bytes.empty() && ortho_decodes >= ORTHO_DECODES_PER_FRAME) {
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_back(std::move(result));
			// Stop the loop entirely — the next frame will pick up where
			// we left off. (push_back so non-ortho tiles still drain.)
			break;
		}

		state.loading = false;

		if (!result.success) {
			state.no_data = true;
			UtilityFunctions::print("S3DTileManager: tile no_data (", result.ei,
				", ", result.ni, ") desired_lod=", state.desired_lod);
			continue;
		}

		int desired_lod = state.desired_lod >= 0 ? state.desired_lod : result.desired_lod;
		int tpx = result.tile_px > 0 ? result.tile_px : result.tile_size;
		int stride = 1 << S3DTile::effective_lod(desired_lod, tpx, tile_size);
		int verts_per_axis = (tpx - 1) / stride + 1;
		int tile_verts = verts_per_axis * verts_per_axis;

		if (verts_generated + tile_verts > VERTEX_BUDGET_PER_FRAME && verts_generated > 0) {
			state.loading = true;
			std::lock_guard<std::mutex> lock(results_mutex);
			results_queue.push_front(std::move(result));
			break;
		}

		PackedByteArray bytes;
		bytes.resize(result.raw_bytes.size());
		memcpy(bytes.ptrw(), result.raw_bytes.data(), result.raw_bytes.size());
		result.raw_bytes.clear();
		if (!terrain_patches.empty() && bytes.size() == (int64_t)tpx * tpx * 4) {
			// the scenery design's plateaus and corridors, before the mesh and
			// the elevation DB see the heights
			const double ts = (double)tile_size;
			terrain_patches.apply(reinterpret_cast<float *>(bytes.ptrw()), tpx, tpx,
				(double)(result.ei + 1) * ts, (double)result.ni * ts, ts);
		}

		Ref<Image> heightmap = Image::create_from_data(
			tpx, tpx, false, Image::FORMAT_RF, bytes);
		if (heightmap.is_null()) {
			state.no_data = true;
			continue;
		}

		if (!state.node) {
			S3DTile *tile = memnew(S3DTile);
			tile->set_tile_x(result.ei);
			tile->set_tile_z(result.ni);
			tile->set_tile_size(result.tile_size);
			tile->set_material(shared_material);

			const double ts = (double)tile_size;
			tile->set_placement(frame,
				(double)(result.ei + 1) * ts, (double)result.ni * ts,
				((double)result.ei + 0.5) * ts, ((double)result.ni + 0.5) * ts,
				0.0, false);

			add_child(tile);
			state.node = tile;
			bump_coverage();
		}

		// (Re)build the ortho material every time we get fresh JPEG bytes.
		// LOD upgrades (far → close) deliver a higher-resolution mip than what
		// the tile was originally created with, so the material must be
		// refreshed — otherwise tiles loaded at mip2/mip3 stay blurry even
		// after the camera moves into their mip0 range.
		if (!result.ortho_rgb.empty() && result.ortho_rgb_w > 0) {
			// Pre-decoded composite from worker (skip_white_pixels merge of
			// multiple ortho sources). Skip JPEG decode entirely.
			int w = result.ortho_rgb_w;
			PackedByteArray rgb;
			rgb.resize((int)result.ortho_rgb.size());
			memcpy(rgb.ptrw(), result.ortho_rgb.data(), result.ortho_rgb.size());
			result.ortho_rgb.clear();
			result.jpeg_bytes.clear();

			Ref<Image> ortho_img = Image::create_from_data(w, w, false, Image::FORMAT_RGB8, rgb);
			if (ortho_img.is_valid() && ortho_img->get_width() > 1) {
				int target_px = result.ortho_pixels;
				if (target_px > 0 && ortho_img->get_width() > target_px) {
					ortho_img->resize(target_px, target_px, Image::INTERPOLATE_BILINEAR);
				}
				ortho_img->generate_mipmaps();
				Ref<ImageTexture> tex = ImageTexture::create_from_image(ortho_img);
				Ref<StandardMaterial3D> mat;
				mat.instantiate();
				mat->set_texture(StandardMaterial3D::TEXTURE_ALBEDO, tex);
				mat->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
				mat->set_texture_filter(BaseMaterial3D::TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC);
				// Clamp to edge (disable repeat): otherwise the mip/anisotropic
				// sampler wraps past the [0,1] UV edge to the opposite side of
				// the tile texture, bleeding the wrong pixels in as a seam.
				mat->set_flag(BaseMaterial3D::FLAG_USE_TEXTURE_REPEAT, false);
				state.node->set_material(mat);
				state.current_ortho_pixels = result.ortho_pixels;
				ortho_decodes++;
			}
		} else if (!result.jpeg_bytes.empty()) {
			PackedByteArray jpeg_arr;
			jpeg_arr.resize(result.jpeg_bytes.size());
			memcpy(jpeg_arr.ptrw(), result.jpeg_bytes.data(), result.jpeg_bytes.size());
			result.jpeg_bytes.clear();

			Ref<Image> ortho_img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
			Error err = ortho_img->load_jpg_from_buffer(jpeg_arr);
			if (err == OK && ortho_img->get_width() > 1) {
				// Downsample mid-distance tiles in memory so a single mip0
				// source can serve every LOD that needs better than mip2
				// without blowing GPU RAM. Close range keeps full 1024 px.
				int target_px = result.ortho_pixels;
				if (target_px > 0 && ortho_img->get_width() > target_px) {
					ortho_img->resize(target_px, target_px, Image::INTERPOLATE_BILINEAR);
				}
				// Generate mipmaps so oblique / distant viewing angles don't
				// shimmer and so anisotropic filtering has mips to sample from.
				ortho_img->generate_mipmaps();
				Ref<ImageTexture> tex = ImageTexture::create_from_image(ortho_img);
				Ref<StandardMaterial3D> mat;
				mat.instantiate();
				mat->set_texture(StandardMaterial3D::TEXTURE_ALBEDO, tex);
				mat->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
				// Anisotropic filtering + clamp-to-edge: reduces texture
				// blurriness at grazing angles (common in flight views) and
				// prevents edge-pixel bleed from showing up as tile seams.
				mat->set_texture_filter(BaseMaterial3D::TEXTURE_FILTER_LINEAR_WITH_MIPMAPS_ANISOTROPIC);
				// Clamp to edge — the comment above promised this but the flag
				// was never set, so repeat (the default) wrapped edge texels in
				// the mips and produced the visible seams between ortho tiles.
				mat->set_flag(BaseMaterial3D::FLAG_USE_TEXTURE_REPEAT, false);
				state.node->set_material(mat);
				state.current_ortho_pixels = result.ortho_pixels;
				ortho_decodes++;
			}
		}

		state.node->set_heightmap(heightmap);
		state.node->set_lod_level(desired_lod);
		state.node->generate_mesh();
		state.current_lod = desired_lod;
		verts_generated += tile_verts;

		// Populate the elevation DB only for tiles at close LOD: this keeps
		// the heightmap cache bounded (load_tile holds Ref<Image>), and is
		// consistent with the fine↔coarse LOD-transition logic further
		// down. Roads and water that load while their terrain is still at
		// coarse LOD will retry their drape once a fine-LOD terrain tile
		// arrives (see S3D{Road,Water}Manager::retry_pending_drapes).
		if (desired_lod < LOD_DISCARD_THRESHOLD && elevation_db.is_valid()) {
			elevation_db->load_tile(result.ei, result.ni, heightmap);
		}
		// Every loaded tile also leaves a coarse copy (<= 65 px, 4 KB) in the
		// database: the vegetation stands on the terrain out to the terrain
		// radius instead of only inside the fine-LOD box (trees popped up at
		// its edge, gery 2026-11-08). Roads and water keep draping on the
		// fine tiles only.
		if (elevation_db.is_valid()) {
			Ref<Image> coarse_hm = heightmap;
			if (heightmap->get_width() > 65 || heightmap->get_height() > 65) {
				coarse_hm = heightmap->duplicate();
				coarse_hm->resize(65, 65, Image::INTERPOLATE_BILINEAR);
			}
			elevation_db->load_coarse_tile(result.ei, result.ni, coarse_hm);
		}

		// Discard the heightmap of distant tiles to bound memory — only
		// worth it for large tiles (a 1025 px tile is 4 MB; a 34 px base
		// tile is 4.6 KB and re-reading it on every ring crossing would
		// just cost I/O).
		if (desired_lod >= LOD_DISCARD_THRESHOLD && heightmap_is_large(heightmap)) {
			state.node->set_heightmap(Ref<Image>());
		}
	}
}

// --- Bind methods ---

void S3DTileManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("update_tiles", "camera_pos"), &S3DTileManager::update_tiles);
	ClassDB::bind_method(D_METHOD("set_terrain_patches", "patches"), &S3DTileManager::set_terrain_patches);
	ClassDB::bind_method(D_METHOD("get_terrain_patch_count"), &S3DTileManager::get_terrain_patch_count);
	ClassDB::bind_method(D_METHOD("reload_tiles"), &S3DTileManager::reload_tiles);
	ClassDB::bind_method(D_METHOD("get_reload_count"), &S3DTileManager::get_reload_count);
	ClassDB::bind_method(D_METHOD("get_active_tile_count"), &S3DTileManager::get_active_tile_count);
	ClassDB::bind_method(D_METHOD("get_pending_count"), &S3DTileManager::get_pending_count);

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DTileManager::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DTileManager::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

	ClassDB::bind_method(D_METHOD("set_tile_px", "px"), &S3DTileManager::set_tile_px);
	ClassDB::bind_method(D_METHOD("get_tile_px"), &S3DTileManager::get_tile_px);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_px"), "set_tile_px", "get_tile_px");

	ClassDB::bind_method(D_METHOD("set_load_radius", "radius"), &S3DTileManager::set_load_radius);
	ClassDB::bind_method(D_METHOD("get_load_radius"), &S3DTileManager::get_load_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_radius"), "set_load_radius", "get_load_radius");

	ClassDB::bind_method(D_METHOD("set_far_radius", "radius"), &S3DTileManager::set_far_radius);
	ClassDB::bind_method(D_METHOD("get_far_radius"), &S3DTileManager::get_far_radius);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "far_radius"), "set_far_radius", "get_far_radius");

	ClassDB::bind_method(D_METHOD("set_load_budget", "budget"), &S3DTileManager::set_load_budget);
	ClassDB::bind_method(D_METHOD("get_load_budget"), &S3DTileManager::get_load_budget);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "load_budget"), "set_load_budget", "get_load_budget");

	ClassDB::bind_method(D_METHOD("set_unload_margin", "margin"), &S3DTileManager::set_unload_margin);
	ClassDB::bind_method(D_METHOD("get_unload_margin"), &S3DTileManager::get_unload_margin);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "unload_margin"), "set_unload_margin", "get_unload_margin");

	ClassDB::bind_method(D_METHOD("get_coverage_mask"), &S3DTileManager::get_coverage_mask);
	ClassDB::bind_method(D_METHOD("get_coverage_mask_radius"), &S3DTileManager::get_coverage_mask_radius);
	ClassDB::bind_method(D_METHOD("get_cam_tile_ei"), &S3DTileManager::get_cam_tile_ei);
	ClassDB::bind_method(D_METHOD("get_cam_tile_ni"), &S3DTileManager::get_cam_tile_ni);
	ClassDB::bind_method(D_METHOD("get_chunk_size"), &S3DTileManager::get_chunk_size);
	ClassDB::bind_method(D_METHOD("get_coverage_generation"), &S3DTileManager::get_coverage_generation);

	ClassDB::bind_method(D_METHOD("set_data_path", "path"), &S3DTileManager::set_data_path);
	ClassDB::bind_method(D_METHOD("get_data_path"), &S3DTileManager::get_data_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "data_path"), "set_data_path", "get_data_path");

	ClassDB::bind_method(D_METHOD("set_data_paths", "paths"), &S3DTileManager::set_data_paths);
	ClassDB::bind_method(D_METHOD("get_data_paths"), &S3DTileManager::get_data_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "data_paths"), "set_data_paths", "get_data_paths");

	ClassDB::bind_method(D_METHOD("set_origin_east", "east"), &S3DTileManager::set_origin_east);
	ClassDB::bind_method(D_METHOD("get_origin_east"), &S3DTileManager::get_origin_east);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_east"), "set_origin_east", "get_origin_east");

	ClassDB::bind_method(D_METHOD("set_origin_north", "north"), &S3DTileManager::set_origin_north);
	ClassDB::bind_method(D_METHOD("get_origin_north"), &S3DTileManager::get_origin_north);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "origin_north"), "set_origin_north", "get_origin_north");

	ClassDB::bind_method(D_METHOD("set_frame_mode", "mode"), &S3DTileManager::set_frame_mode);
	ClassDB::bind_method(D_METHOD("get_frame_mode"), &S3DTileManager::get_frame_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "frame_mode"), "set_frame_mode", "get_frame_mode");

	ClassDB::bind_method(D_METHOD("set_orthophoto_path", "path"), &S3DTileManager::set_orthophoto_path);
	ClassDB::bind_method(D_METHOD("get_orthophoto_path"), &S3DTileManager::get_orthophoto_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "orthophoto_path"), "set_orthophoto_path", "get_orthophoto_path");

	ClassDB::bind_method(D_METHOD("set_orthophoto_paths", "paths"), &S3DTileManager::set_orthophoto_paths);
	ClassDB::bind_method(D_METHOD("get_orthophoto_paths"), &S3DTileManager::get_orthophoto_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "orthophoto_paths"), "set_orthophoto_paths", "get_orthophoto_paths");

	ClassDB::bind_method(D_METHOD("set_landcover_paths", "paths"), &S3DTileManager::set_landcover_paths);
	ClassDB::bind_method(D_METHOD("get_landcover_paths"), &S3DTileManager::get_landcover_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "landcover_paths"), "set_landcover_paths", "get_landcover_paths");
	ClassDB::bind_method(D_METHOD("landcover_image", "path", "px"), &S3DTileManager::landcover_image);
	ClassDB::bind_method(D_METHOD("chunk_cover_covers", "ei", "ni"), &S3DTileManager::chunk_cover_covers);
	ClassDB::bind_method(D_METHOD("get_masked_chunk_count"), &S3DTileManager::get_masked_chunk_count);

	ClassDB::bind_method(D_METHOD("set_skip_white_pixels", "skip"), &S3DTileManager::set_skip_white_pixels);
	ClassDB::bind_method(D_METHOD("get_skip_white_pixels"), &S3DTileManager::get_skip_white_pixels);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "skip_white_pixels"), "set_skip_white_pixels", "get_skip_white_pixels");
}

// --- _process ---

void S3DTileManager::_process(double delta)
{
	Viewport *vp = get_viewport();
	if (!vp) return;

	Camera3D *camera = vp->get_camera_3d();
	if (!camera) return;

	// Start worker threads on first frame.
	if (!worker_running.load()) {
		start_worker();
	}

	update_tiles(camera->get_global_position());
}

// --- Main update logic ---

void S3DTileManager::update_tiles(Vector3 camera_pos)
{
	if (lod_rings.empty()) return;

	// Convert camera world position to grid (LV95) tile indices through the
	// world frame (flat: origin offset; ECEF: geodetic + CRS forward).
	const proj::EN cam_grid = frame.world_to_grid(camera_pos);
	double cam_lv95_e = cam_grid.e;
	double cam_lv95_n = cam_grid.n;
	int cam_ei = (int)std::floor(cam_lv95_e / (double)tile_size);
	int cam_ni = (int)std::floor(cam_lv95_n / (double)tile_size);

	// Remember for the coverage query (get_coverage_mask is centred on this).
	last_cam_ei = cam_ei;
	last_cam_ni = cam_ni;
	if (chunk_size >= 1) {
		last_cam_cg_ei = (cam_ei >= 0)
			? cam_ei / chunk_size
			: (cam_ei - chunk_size + 1) / chunk_size;
		last_cam_cg_ni = (cam_ni >= 0)
			? cam_ni / chunk_size
			: (cam_ni - chunk_size + 1) / chunk_size;
	}

	int max_radius = lod_rings.back().radius;

	// 1. Compute desired LOD for all tiles in range and track required set.
	std::unordered_set<uint64_t> required;
	std::vector<LoadRequest> new_requests;

	for (int dei = -max_radius; dei <= max_radius; dei++) {
		for (int dni = -max_radius; dni <= max_radius; dni++) {
			int dist = std::max(std::abs(dei), std::abs(dni));
			if (dist > max_radius) continue;

			int ei = cam_ei + dei;
			int ni = cam_ni + dni;
			uint64_t key = tile_key(ei, ni);
			int desired_lod = lod_for_distance(dist);

			required.insert(key);

			auto it = tiles.find(key);
			if (it == tiles.end()) {
				// New tile — create state and queue file load.
				TileState state;
				state.desired_lod = desired_lod;
				state.loading = true;
				tiles[key] = state;

				int lv95_east = ei * tile_size;
				int lv95_north = ni * tile_size;
				String tname = "/tile_" + itos(lv95_east) + "_" + itos(lv95_north) + ".raw";

				LoadRequest req;
				req.ei = ei;
				req.ni = ni;
				req.key = key;
				for (int i = 0; i < data_paths.size(); i++) {
					req.paths.push_back(std::string((data_paths[i] + tname).utf8().get_data()));
					std::string key(data_paths[i].utf8().get_data());
					auto px_it = path_tile_px.find(key);
					req.path_px.push_back(px_it != path_tile_px.end() ? px_it->second : 0);
				}
				req.tile_size = tile_size;
				req.tile_px = effective_tile_px();
				req.desired_lod = desired_lod;
				req.distance = dist;
				req.ortho_paths = ortho_paths_for_tile(ei, ni, desired_lod);
				req.ortho_pixels = req.ortho_paths.empty() ? -1 : ortho_texture_pixels_for_lod(desired_lod);
				req.landcover_paths = landcover_paths_for_tile(ei, ni);
				new_requests.push_back(std::move(req));
			} else {
				// Existing tile — update desired LOD.
				it->second.desired_lod = desired_lod;
			}
		}
	}

	// 2. Unload tiles outside range (with hysteresis margin).
	// For tiles entering the chunk zone, only remove if the covering chunk
	// is loaded so there's no gap during the handoff.
	int unload_radius = max_radius + unload_margin;
	bool have_chunks = (far_radius > max_radius && chunk_size >= 1);
	std::vector<uint64_t> to_remove;
	for (auto &[key, state] : tiles) {
		int ei = (int)(int32_t)(key >> 32);
		int ni = (int)(int32_t)(key & 0xFFFFFFFF);
		int dist = std::max(std::abs(ei - cam_ei), std::abs(ni - cam_ni));
		if (dist > unload_radius) {
			if (have_chunks && dist <= far_radius) {
				// Tile is in chunk zone — only defer unload while the
				// covering chunk is still loading (to avoid a visible gap
				// during the close→far handoff). If the chunk is either
				// already loaded OR not tracked at all (camera moved on,
				// chunk aged out), drop the tile. Without the
				// "not tracked" case, stale close-ring tiles from a
				// previous airport pile up indefinitely after each
				// teleport — driving `tiles.size()` from ~10k to >30k
				// across a handful of jumps and tanking framerate.
				int cg_ei = (ei >= 0) ? ei / chunk_size : (ei - chunk_size + 1) / chunk_size;
				int cg_ni = (ni >= 0) ? ni / chunk_size : (ni - chunk_size + 1) / chunk_size;
				uint64_t ckey = tile_key(cg_ei, cg_ni);
				auto cit = chunks.find(ckey);
				if (cit == chunks.end()) {
					to_remove.push_back(key);
				} else if (!cit->second.loading && cit->second.node) {
					to_remove.push_back(key);
				}
				// else: chunk is currently loading — defer one frame.
			} else {
				to_remove.push_back(key);
			}
		}
	}

	for (uint64_t key : to_remove) {
		auto it = tiles.find(key);
		if (it != tiles.end()) {
			TileState &state = it->second;
			if (state.node) {
				if (elevation_db.is_valid()) {
					elevation_db->unload_tile(state.node->get_tile_x(), state.node->get_tile_z());
					elevation_db->unload_coarse_tile(state.node->get_tile_x(), state.node->get_tile_z());
				}
				state.node->queue_free();
				bump_coverage();
			}
			tiles.erase(it);
		}
	}

	// 3. Process completed file loads from worker thread (budget-limited).
	int verts_generated = 0;
	process_load_results(verts_generated);

	// 4. Handle LOD transitions for existing tiles.
	for (auto &[key, state] : tiles) {
		if (verts_generated >= VERTEX_BUDGET_PER_FRAME) break;

		if (state.node && !state.loading && !state.no_data &&
			state.desired_lod != state.current_lod &&
			state.desired_lod >= 0) {

			// Decide whether the LOD change crosses a texture-resolution
			// boundary. If it does we can't just re-mesh in place — we need
			// to fetch a fresh ortho so the tile doesn't stay stuck at the
			// (often coarser) resolution it was first loaded with.
			int desired_pixels = -1;
			if (!orthophoto_paths.is_empty()) {
				desired_pixels = ortho_texture_pixels_for_lod(state.desired_lod);
			}
			bool mip_change = (desired_pixels != -1 && desired_pixels != state.current_ortho_pixels);

			if (state.node->get_heightmap().is_valid() && !mip_change) {
				// Budget against the heightmap's actual resolution — with
				// per-path pixels_per_tile (Norway 34 px vs CH/BW 512 px)
				// the node-wide value would over-count ~200×.
				int img_w = state.node->get_heightmap()->get_width();
				int eff_desired = S3DTile::effective_lod(state.desired_lod, img_w, tile_size);
				int eff_current = S3DTile::effective_lod(state.current_lod, img_w, tile_size);
				int stride = 1 << eff_desired;
				int verts_per_axis = (img_w - 1) / stride + 1;
				int tile_verts = 0;

				state.node->set_lod_level(state.desired_lod);
				// A ring change that maps to the same stride for this
				// tile's resolution keeps the mesh: no rebuild, no pop.
				if (eff_desired != eff_current || state.current_lod < 0) {
					state.node->generate_mesh();
					tile_verts = verts_per_axis * verts_per_axis;
				}

				// Update elevation DB registration.
				if (state.current_lod >= LOD_DISCARD_THRESHOLD &&
					state.desired_lod < LOD_DISCARD_THRESHOLD &&
					elevation_db.is_valid()) {
					elevation_db->load_tile(
						state.node->get_tile_x(),
						state.node->get_tile_z(),
						state.node->get_heightmap());
				} else if (state.current_lod < LOD_DISCARD_THRESHOLD &&
						   state.desired_lod >= LOD_DISCARD_THRESHOLD &&
						   elevation_db.is_valid()) {
					elevation_db->unload_tile(
						state.node->get_tile_x(),
						state.node->get_tile_z());
				}

				state.current_lod = state.desired_lod;
				verts_generated += tile_verts;

				// Discard heightmap if now distant (large tiles only, see
				// heightmap_is_large).
				if (state.desired_lod >= LOD_DISCARD_THRESHOLD && heightmap_is_large(state.node->get_heightmap())) {
					state.node->set_heightmap(Ref<Image>());
				}
			} else {
				// Either no cached heightmap, or the ortho mip level needs
				// to change — re-read from disk in both cases.
				if (!state.loading) {
					state.loading = true;
					int ei = state.node->get_tile_x();
					int ni = state.node->get_tile_z();
					int lv95_east = ei * tile_size;
					int lv95_north = ni * tile_size;
					String tname = "/tile_" + itos(lv95_east) + "_" + itos(lv95_north) + ".raw";

					LoadRequest req;
					req.ei = ei;
					req.ni = ni;
					req.key = key;
					for (int i = 0; i < data_paths.size(); i++) {
						req.paths.push_back(std::string((data_paths[i] + tname).utf8().get_data()));
						std::string key(data_paths[i].utf8().get_data());
						auto px_it = path_tile_px.find(key);
						req.path_px.push_back(px_it != path_tile_px.end() ? px_it->second : 0);
					}
					req.tile_size = tile_size;
					req.tile_px = effective_tile_px();
					req.desired_lod = state.desired_lod;
					req.distance = 0; // High priority for LOD upgrades.
					req.ortho_paths = ortho_paths_for_tile(ei, ni, state.desired_lod);
					req.ortho_pixels = req.ortho_paths.empty() ? -1 : ortho_texture_pixels_for_lod(state.desired_lod);
					req.landcover_paths = landcover_paths_for_tile(ei, ni);
					new_requests.push_back(std::move(req));
				}
			}
		}
	}

	// 5. Sort new requests by distance (closest first) and queue to worker.
	//    Then re-prioritise the entire work_queue based on the *current*
	//    camera position: a request queued frames ago against an older
	//    camera position may now be far away, while new close-LOD requests
	//    must not wait behind it. Without this, fast camera motion creates
	//    a permanent priority inversion (the queue keeps grinding through
	//    stale far-LOD tiles while the close ring stays empty).
	{
		std::lock_guard<std::mutex> lock(work_mutex);
		for (auto &req : new_requests) {
			work_queue.push_back(std::move(req));
		}

		// Drop queued requests whose tile/chunk entry has been unloaded.
		// Chunk requests live in the `chunks` map (keyed by chunk-grid index),
		// regular tile requests in `tiles`. The predicate MUST check the
		// matching map: a previous version always checked `tiles`, so every
		// queued chunk request — whose key is a chunk-grid index never present
		// in `tiles` — was erased one frame after being queued, before any
		// worker could process it. That silently disabled the entire far
		// terrain panorama.
		work_queue.erase(
			std::remove_if(work_queue.begin(), work_queue.end(),
				[this](const LoadRequest &req) {
					if (req.is_chunk) {
						return chunks.find(req.key) == chunks.end();
					}
					return tiles.find(req.key) == tiles.end();
				}),
			work_queue.end());

		// Recompute distance against current camera and sort closest-first.
		// Chunk requests keep their own distance (set from the closest tile in
		// the chunk): their req.ei/ni are chunk-grid indices, not tile indices,
		// so recomputing against cam_ei/ni here would assign a meaningless
		// (huge) distance and bury them at the back of the queue forever.
		for (auto &req : work_queue) {
			if (req.is_chunk) continue;
			int de = req.ei - cam_ei;
			int dn = req.ni - cam_ni;
			req.distance = std::max(std::abs(de), std::abs(dn));
		}
		std::sort(work_queue.begin(), work_queue.end(),
			[](const LoadRequest &a, const LoadRequest &b) {
				return a.distance < b.distance;
			});

		if (!new_requests.empty()) {
			work_cv.notify_all();
		}
	}

	// Diagnostic: once-per-second tile state summary.
	{
		static uint64_t last_print_ms = 0;
		uint64_t now_ms = Time::get_singleton()->get_ticks_msec();
		if (now_ms - last_print_ms > 1000) {
			last_print_ms = now_ms;
			int loading = 0, no_data = 0, ready_close = 0;
			std::vector<std::pair<int, int>> ready_coords;
			std::vector<std::pair<int, int>> nodata_close;
			for (auto &kv : tiles) {
				if (kv.second.loading) loading++;
				if (kv.second.no_data) {
					no_data++;
					// Track no_data tiles inside the close ring (dist<=3).
					int ei = (int)(int32_t)(kv.first >> 32);
					int ni = (int)(int32_t)(kv.first & 0xFFFFFFFF);
					int d = std::max(std::abs(ei - cam_ei), std::abs(ni - cam_ni));
					if (d <= 3 && nodata_close.size() < 10) {
						nodata_close.emplace_back(ei, ni);
					}
				}
				if (kv.second.node && !kv.second.loading
					&& kv.second.current_lod >= 0
					&& kv.second.current_lod < LOD_DISCARD_THRESHOLD) {
					ready_close++;
					if (ready_coords.size() < 12) {
						int ei = (int)(int32_t)(kv.first >> 32);
						int ni = (int)(int32_t)(kv.first & 0xFFFFFFFF);
						ready_coords.emplace_back(ei, ni);
					}
				}
			}
			int elev_size = elevation_db.is_valid() ? elevation_db->get_tile_count() : -1;
			UtilityFunctions::print("S3DTileManager[", (uint64_t)this, "]: cam=(", cam_ei, ",", cam_ni,
				") tiles=", (int)tiles.size(), " loading=", loading,
				" no_data=", no_data, " ready<LOD", (int)LOD_DISCARD_THRESHOLD,
				"=", ready_close, " elev_db=", elev_size,
				" chunks=", (int)chunks.size(),
				" workq=", (int)work_queue.size(),
				" data_paths=", (int)data_paths.size());
			// Sample of which tiles are currently ready.
			String rs;
			for (auto &p : ready_coords) {
				rs += " (" + itos(p.first) + "," + itos(p.second) + ")";
			}
			UtilityFunctions::print("    ready_tiles:", rs);
			// Sample of close-ring no_data tiles (dist<=3 from camera).
			if (!nodata_close.empty()) {
				String ns;
				for (auto &p : nodata_close) {
					ns += " (" + itos(p.first) + "," + itos(p.second) + ")";
				}
				UtilityFunctions::print("    close_no_data:", ns);
			}
			// Echo data_paths so we can verify the project setting reached
			// the manager.
			for (int i = 0; i < data_paths.size(); i++) {
				UtilityFunctions::print("    data_path[", i, "]=", data_paths[i]);
			}
		}
	}

	// ===== FAR TERRAIN CHUNKS =====
	if (far_radius <= max_radius || chunk_size < 1) return;

	int far_chunk_radius = (far_radius + chunk_size - 1) / chunk_size;
	int cam_cg_ei = (cam_ei >= 0)
		? cam_ei / chunk_size
		: (cam_ei - chunk_size + 1) / chunk_size;
	int cam_cg_ni = (cam_ni >= 0)
		? cam_ni / chunk_size
		: (cam_ni - chunk_size + 1) / chunk_size;

	std::unordered_set<uint64_t> required_chunks;
	std::vector<LoadRequest> chunk_requests;
	std::vector<std::string> base_paths_str;
	for (int i = 0; i < data_paths.size(); i++) {
		String p = data_paths[i];
		if (!p.is_empty()) {
			base_paths_str.push_back(std::string(p.utf8().get_data()));
		}
	}

	for (int dcg_ei = -far_chunk_radius; dcg_ei <= far_chunk_radius; dcg_ei++) {
		for (int dcg_ni = -far_chunk_radius; dcg_ni <= far_chunk_radius; dcg_ni++) {
			int cg_ei = cam_cg_ei + dcg_ei;
			int cg_ni = cam_cg_ni + dcg_ni;

			// Compute tile distances from this chunk to camera.
			int base_ei = cg_ei * chunk_size;
			int base_ni = cg_ni * chunk_size;
			int closest_ei = std::max(base_ei, std::min(cam_ei, base_ei + chunk_size - 1));
			int closest_ni = std::max(base_ni, std::min(cam_ni, base_ni + chunk_size - 1));
			int min_dist = std::max(std::abs(closest_ei - cam_ei), std::abs(closest_ni - cam_ni));

			// Max distance: farthest tile in chunk from camera.
			int max_dei = std::max(std::abs(base_ei - cam_ei), std::abs(base_ei + chunk_size - 1 - cam_ei));
			int max_dni = std::max(std::abs(base_ni - cam_ni), std::abs(base_ni + chunk_size - 1 - cam_ni));
			int max_dist = std::max(max_dei, max_dni);

			// Only skip if ALL tiles in chunk are covered by individual tiles.
			if (max_dist <= max_radius) continue;
			// Skip chunks beyond far radius (use closest tile).
			if (min_dist > far_radius) continue;

			uint64_t ckey = tile_key(cg_ei, cg_ni);
			required_chunks.insert(ckey);

			auto it = chunks.find(ckey);
			bool need_request = false;
			int cres = CHUNK_COMPOSITE_RES;
			if (it == chunks.end()) {
				TileState state;
				state.desired_lod = 0;
				state.loading = true;
				chunks[ckey] = state;
				cres = chunk_res_for_distance(min_dist, 0);
				need_request = true;
			} else if (!it->second.loading && !it->second.no_data && it->second.node != nullptr) {
				cres = chunk_res_for_distance(min_dist, it->second.chunk_res);
				if (cres != it->second.chunk_res) {
					it->second.loading = true;
					need_request = true;
				}
			}
			if (need_request) {
				LoadRequest req;
				req.is_chunk = true;
				req.ei = cg_ei;
				req.ni = cg_ni;
				req.key = ckey;
				req.chunk_grid_ei = cg_ei;
				req.chunk_grid_ni = cg_ni;
				req.chunk_tile_count = chunk_size;
				req.composite_res = cres;
				req.tile_size = tile_size;
				req.tile_px = effective_tile_px();
				req.base_paths = base_paths_str;
				for (const auto &bp : base_paths_str) {
					auto px_it = path_tile_px.find(bp);
					req.base_path_px.push_back(px_it != path_tile_px.end() ? px_it->second : 0);
				}
				req.distance = min_dist;
				for (int i = 0; i < orthophoto_paths.size(); i++) {
					String p = orthophoto_paths[i];
					if (p.is_empty()) continue;
					req.ortho_mip_dirs.push_back(std::string(p.utf8().get_data()) + "/mip3");
				}
				for (int i = 0; i < landcover_paths.size(); i++) {
					String p = landcover_paths[i];
					if (p.is_empty()) continue;
					req.landcover_dirs.push_back(std::string(p.utf8().get_data()));
				}
				chunk_requests.push_back(std::move(req));
			}
		}
	}

	// Unload out-of-range chunks (with hysteresis margin).
	// A chunk may be dropped only once every individual tile within its
	// footprint that the camera can currently see (i.e. inside max_radius)
	// has finished loading — otherwise we'd punch a hole in the world
	// while the streamer catches up. The Y-bias hides Z-fighting where
	// chunks and tiles overlap, but it can't fill a gap that has neither.
	int chunk_unload_far = far_radius + unload_margin;
	std::vector<uint64_t> chunks_to_remove;
	for (auto &[key, state] : chunks) {
		int cg_ei = (int)(int32_t)(key >> 32);
		int cg_ni = (int)(int32_t)(key & 0xFFFFFFFF);
		int base_ei = cg_ei * chunk_size;
		int base_ni = cg_ni * chunk_size;
		int closest_ei = std::max(base_ei, std::min(cam_ei, base_ei + chunk_size - 1));
		int closest_ni = std::max(base_ni, std::min(cam_ni, base_ni + chunk_size - 1));
		int min_dist = std::max(std::abs(closest_ei - cam_ei), std::abs(closest_ni - cam_ni));
		int max_dei = std::max(std::abs(base_ei - cam_ei), std::abs(base_ei + chunk_size - 1 - cam_ei));
		int max_dni = std::max(std::abs(base_ni - cam_ni), std::abs(base_ni + chunk_size - 1 - cam_ni));
		int max_dist_in_chunk = std::max(max_dei, max_dni);
		if (min_dist > chunk_unload_far) {
			chunks_to_remove.push_back(key);
			continue;
		}
		if (max_dist_in_chunk <= max_radius) {
			// Footprint is fully inside the individual-tile ring. Only
			// drop the chunk if every covered tile is actually present
			// (loaded node, or marked no_data). If any are still in
			// flight, keep the chunk so the area stays filled.
			bool all_ready = true;
			for (int ldy = 0; ldy < chunk_size && all_ready; ldy++) {
				for (int ldx = 0; ldx < chunk_size && all_ready; ldx++) {
					int tei = base_ei + ldx;
					int tni = base_ni + ldy;
					int td = std::max(std::abs(tei - cam_ei), std::abs(tni - cam_ni));
					if (td > max_radius) continue; // tile outside ring; ignore
					auto tit = tiles.find(tile_key(tei, tni));
					if (tit == tiles.end()) { all_ready = false; break; }
					const TileState &ts = tit->second;
					if (ts.no_data) continue;
					if (!ts.node || ts.loading || ts.current_lod < 0) {
						all_ready = false;
					}
				}
			}
			if (all_ready) {
				chunks_to_remove.push_back(key);
			}
		}
	}
	update_chunk_cover(cam_ei, cam_ni, max_radius);

	for (uint64_t key : chunks_to_remove) {
		auto it = chunks.find(key);
		if (it != chunks.end()) {
			if (it->second.node) {
				it->second.node->queue_free();
				bump_coverage();
			}
			chunks.erase(it);
		}
	}

	// Queue chunk requests (low priority — after individual tiles).
	if (!chunk_requests.empty()) {
		std::sort(chunk_requests.begin(), chunk_requests.end(),
			[](const LoadRequest &a, const LoadRequest &b) {
				return a.distance < b.distance;
			});

		std::lock_guard<std::mutex> lock(work_mutex);
		for (auto &req : chunk_requests) {
			work_queue.push_back(std::move(req));
		}
		work_cv.notify_all();
	}
}

// --- Getters / Setters ---

int S3DTileManager::get_pending_count()
{
	int n = 0;
	for (const auto &kv : tiles) if (kv.second.loading) n++;
	for (const auto &kv : chunks) if (kv.second.loading) n++;
	{
		std::lock_guard<std::mutex> lock(work_mutex);
		n += (int)work_queue.size();
	}
	return n;
}

int S3DTileManager::get_active_tile_count() const
{
	return (int)tiles.size() + (int)chunks.size();
}

void S3DTileManager::bump_coverage()
{
	coverage_generation++;
}

PackedByteArray S3DTileManager::get_coverage_mask() const
{
	const int r = get_coverage_mask_radius();
	const int n = 2 * r + 1;
	PackedByteArray out;
	out.resize((int64_t)n * n);
	out.fill(0);
	uint8_t *w = out.ptrw();
	const int e_min = last_cam_ei - r;
	const int n_min = last_cam_ni - r;
	auto stamp = [&](int ei, int ni) {
		const int ie = ei - e_min;
		const int in = ni - n_min;
		if (ie >= 0 && ie < n && in >= 0 && in < n) {
			w[(int64_t)in * n + ie] = 255;
		}
	};
	// Far zone: a resident chunk mesh spans its whole footprint (partial
	// chunks are gap-filled at bake time, so the geometry really is there).
	// A failed chunk stays in the map with no_data set and node == null —
	// not covered, so the coarse globe fills it instead of leaving a hole.
	if (chunk_size >= 1) {
		for (const auto &[key, state] : chunks) {
			if (!state.node || state.node->get_mesh().is_null()) continue;
			const int cg_ei = (int)(int32_t)(key >> 32);
			const int cg_ni = (int)(int32_t)(key & 0xFFFFFFFF);
			for (int dy = 0; dy < chunk_size; dy++) {
				for (int dx = 0; dx < chunk_size; dx++) {
					stamp(cg_ei * chunk_size + dx, cg_ni * chunk_size + dy);
				}
			}
		}
	}
	// Individual tiles (close ring, and stale far tiles still waiting for
	// their chunk). A tile mid-LOD-reload keeps its old node and mesh, so
	// "node with a mesh" is exactly "geometry drawn"; a no_data tile has
	// none and stays 0, and so does a node whose generate_mesh() bailed.
	for (const auto &[key, state] : tiles) {
		if (!state.node || state.node->get_mesh().is_null()) continue;
		stamp((int)(int32_t)(key >> 32), (int)(int32_t)(key & 0xFFFFFFFF));
	}
	return out;
}

void S3DTileManager::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DTileManager::get_tile_size() const
{
	return tile_size;
}

void S3DTileManager::set_tile_px(int p_px)
{
	tile_px = p_px;
}

int S3DTileManager::get_tile_px() const
{
	return tile_px;
}

void S3DTileManager::set_load_radius(int p_radius)
{
	load_radius = p_radius;
	rebuild_lod_rings();
}

int S3DTileManager::get_load_radius() const
{
	return load_radius;
}

void S3DTileManager::set_far_radius(int p_radius)
{
	far_radius = p_radius;
}

int S3DTileManager::get_far_radius() const
{
	return far_radius;
}

void S3DTileManager::set_load_budget(int p_budget)
{
	load_budget = p_budget;
}

int S3DTileManager::get_load_budget() const
{
	return load_budget;
}

void S3DTileManager::set_unload_margin(int p_margin)
{
	unload_margin = p_margin;
}

int S3DTileManager::get_unload_margin() const
{
	return unload_margin;
}

void S3DTileManager::set_data_path(const String &p_path)
{
	data_paths.clear();
	if (!p_path.is_empty()) {
		data_paths.push_back(p_path);
	}
	rebuild_path_tile_px();
}

String S3DTileManager::get_data_path() const
{
	return data_paths.is_empty() ? String() : String(data_paths[0]);
}

void S3DTileManager::set_terrain_patches(const Dictionary &p_patches)
{
	S3DTerrainPatches next;
	Array plateaus = p_patches.get("plateaus", Array());
	for (int i = 0; i < plateaus.size(); i++) {
		Dictionary d = plateaus[i];
		PackedVector2Array poly = d.get("poly", PackedVector2Array());
		if (poly.size() < 3) continue;
		S3DPlateau p;
		p.h = (double)d.get("h", 0.0);
		p.e0 = p.n0 = 1e18; p.e1 = p.n1 = -1e18;
		for (int k = 0; k < poly.size(); k++) {
			p.poly.emplace_back(poly[k].x, poly[k].y);
			p.e0 = std::min(p.e0, (double)poly[k].x); p.e1 = std::max(p.e1, (double)poly[k].x);
			p.n0 = std::min(p.n0, (double)poly[k].y); p.n1 = std::max(p.n1, (double)poly[k].y);
		}
		next.plateaus.push_back(std::move(p));
	}
	Array corridors = p_patches.get("corridors", Array());
	for (int i = 0; i < corridors.size(); i++) {
		Dictionary d = corridors[i];
		PackedVector3Array st = d.get("stations", PackedVector3Array());
		if (st.size() < 2) continue;
		S3DCorridor c;
		c.reach = (double)d.get("reach", 20.0);
		c.half_width = (double)d.get("half", 0.0);
		c.e0 = c.n0 = 1e18; c.e1 = c.n1 = -1e18;
		for (int k = 0; k < st.size(); k++) {
			c.st.push_back({(double)st[k].x, (double)st[k].y, (double)st[k].z});
			c.e0 = std::min(c.e0, (double)st[k].x); c.e1 = std::max(c.e1, (double)st[k].x);
			c.n0 = std::min(c.n0, (double)st[k].y); c.n1 = std::max(c.n1, (double)st[k].y);
		}
		c.e0 -= c.reach; c.e1 += c.reach; c.n0 -= c.reach; c.n1 += c.reach;
		next.corridors.push_back(std::move(c));
	}
	Array pp = p_patches.get("protect_polys", Array());
	for (int i = 0; i < pp.size(); i++) {
		PackedVector2Array poly = pp[i];
		if (poly.size() < 3) continue;
		S3DPlateau p;
		p.e0 = p.n0 = 1e18; p.e1 = p.n1 = -1e18;
		for (int k = 0; k < poly.size(); k++) {
			p.poly.emplace_back(poly[k].x, poly[k].y);
			p.e0 = std::min(p.e0, (double)poly[k].x); p.e1 = std::max(p.e1, (double)poly[k].x);
			p.n0 = std::min(p.n0, (double)poly[k].y); p.n1 = std::max(p.n1, (double)poly[k].y);
		}
		next.protect_polys.push_back(std::move(p));
	}
	Array pc = p_patches.get("protect_corridors", Array());
	for (int i = 0; i < pc.size(); i++) {
		Dictionary d = pc[i];
		PackedVector2Array st = d.get("stations", PackedVector2Array());
		if (st.size() < 2) continue;
		S3DCorridor c;
		c.reach = (double)d.get("reach", 20.0);
		c.half_width = (double)d.get("half", 0.0);
		c.e0 = c.n0 = 1e18; c.e1 = c.n1 = -1e18;
		for (int k = 0; k < st.size(); k++) {
			c.st.push_back({(double)st[k].x, (double)st[k].y, 0.0});
			c.e0 = std::min(c.e0, (double)st[k].x); c.e1 = std::max(c.e1, (double)st[k].x);
			c.n0 = std::min(c.n0, (double)st[k].y); c.n1 = std::max(c.n1, (double)st[k].y);
		}
		c.e0 -= c.reach; c.e1 += c.reach; c.n0 -= c.reach; c.n1 += c.reach;
		next.protect_corridors.push_back(std::move(c));
	}
	terrain_patches = std::move(next);
	reload_tiles();
}

int S3DTileManager::get_terrain_patch_count() const
{
	return (int)(terrain_patches.plateaus.size() + terrain_patches.corridors.size());
}

void S3DTileManager::reload_tiles()
{
	reload_count++;
	for (auto &kv : tiles) {
		TileState &state = kv.second;
		if (state.node) {
			if (elevation_db.is_valid()) {
				elevation_db->unload_tile(state.node->get_tile_x(), state.node->get_tile_z());
				elevation_db->unload_coarse_tile(state.node->get_tile_x(), state.node->get_tile_z());
			}
			state.node->queue_free();
		}
	}
	tiles.clear();
	bump_coverage();
}

void S3DTileManager::set_data_paths(const PackedStringArray &p_paths)
{
	// Region CRS contract (design §9 D6): a terrain manifest may declare its
	// grid CRS ("crs": "EPSG:2056"). The runtime keeps ONE tile grid, LV95
	// (EPSG:2056); paths that declare another CRS are dropped with a clear
	// message instead of being streamed on the wrong grid. Multi-CRS
	// streaming is the next piece of work once the first such pack exists.
	if (p_paths == data_paths_requested) {
		return; // unchanged (settings saved without touching the library)
	}
	data_paths_requested = p_paths;
	data_paths.clear();
	for (int i = 0; i < p_paths.size(); i++) {
		String p = p_paths[i];
		if (p.is_empty()) continue;
		int epsg = manifest_crs_epsg(std::string(p.utf8().get_data()));
		if (epsg != 0 && epsg != 2056) {
			UtilityFunctions::push_warning("S3DTileManager: region ", p, " declares crs EPSG:", epsg,
				" but this build streams a single LV95 (EPSG:2056) grid — path skipped");
			continue;
		}
		data_paths.push_back(p);
	}
	rebuild_path_tile_px();
}

PackedStringArray S3DTileManager::get_data_paths() const
{
	return data_paths;
}

void S3DTileManager::set_origin_east(double p_east)
{
	origin_east = p_east;
	frame.origin_east = p_east;
}

double S3DTileManager::get_origin_east() const
{
	return origin_east;
}

void S3DTileManager::set_origin_north(double p_north)
{
	origin_north = p_north;
	frame.origin_north = p_north;
}

void S3DTileManager::set_frame_mode(int p_mode)
{
	frame.mode = (p_mode == S3DFrame::ECEF) ? S3DFrame::ECEF : S3DFrame::FLAT;
}

double S3DTileManager::get_origin_north() const
{
	return origin_north;
}

void S3DTileManager::set_orthophoto_path(const String &p_path)
{
	// Compat shim: replace the path list with this single entry.
	orthophoto_paths.clear();
	if (!p_path.is_empty()) orthophoto_paths.push_back(p_path);
}

String S3DTileManager::get_orthophoto_path() const
{
	if (orthophoto_paths.is_empty()) return String();
	return orthophoto_paths[0];
}

void S3DTileManager::set_orthophoto_paths(const PackedStringArray &p_paths)
{
	orthophoto_paths = p_paths;
}

PackedStringArray S3DTileManager::get_orthophoto_paths() const
{
	return orthophoto_paths;
}

void S3DTileManager::set_landcover_paths(const PackedStringArray &p_paths)
{
	landcover_paths = p_paths;
}

PackedStringArray S3DTileManager::get_landcover_paths() const
{
	return landcover_paths;
}

Ref<Material> S3DTileManager::make_chunk_material(int cg_ei, int cg_ni, Ref<Texture2D> p_tex)
{
	if (chunk_shader.is_null()) {
		ResourceLoader *rl = ResourceLoader::get_singleton();
		const String path = "res://addons/scenery3d/shaders/chunk.gdshader";
		if (rl && rl->exists(path)) chunk_shader = rl->load(path);
	}
	if (chunk_shader.is_null()) {
		// No shader (headless without the addon files): the old materials.
		if (p_tex.is_null()) return shared_material;
		Ref<StandardMaterial3D> mat;
		mat.instantiate();
		mat->set_texture(StandardMaterial3D::TEXTURE_ALBEDO, p_tex);
		mat->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
		mat->set_flag(BaseMaterial3D::FLAG_USE_TEXTURE_REPEAT, false);
		return mat;
	}
	Ref<ShaderMaterial> mat;
	mat.instantiate();
	mat->set_shader(chunk_shader);
	mat->set_shader_parameter("chunk_base", Vector2((real_t)(cg_ei * chunk_size), (real_t)(cg_ni * chunk_size)));
	mat->set_shader_parameter("chunk_tiles", (float)chunk_size);
	if (p_tex.is_valid()) {
		mat->set_shader_parameter("albedo_tex", p_tex);
		mat->set_shader_parameter("has_albedo_tex", true);
	}
	return mat;
}

// The cover mask (squares whose individual tile is drawn) for the chunks
// that straddle the close ring; chunks that left the ring zone get their
// mask cleared once. Cheap: the mask is a ~40 px R8 image and only the
// straddling chunks (a handful) are touched.
void S3DTileManager::update_chunk_cover(int cam_ei, int cam_ni, int max_radius)
{
	if (chunk_size < 1 || chunks.empty()) {
		return;
	}
	const int r = max_radius + unload_margin + 1;
	const bool rebuild = coverage_generation != cover_gen_built || cam_ei != cover_cam_ei || cam_ni != cover_cam_ni;
	if (rebuild) {
		const int n = 2 * r + 1;
		PackedByteArray px;
		px.resize((int64_t)n * n);
		px.fill(0);
		uint8_t *w = px.ptrw();
		cover_e_min = cam_ei - r;
		cover_n_min = cam_ni - r;
		cover_n = n;
		for (const auto &[key, state] : tiles) {
			if (!state.node || state.node->get_mesh().is_null()) continue;
			const int ie = (int)(int32_t)(key >> 32) - cover_e_min;
			const int in = (int)(int32_t)(key & 0xFFFFFFFF) - cover_n_min;
			if (ie >= 0 && ie < n && in >= 0 && in < n) w[(int64_t)in * n + ie] = 255;
		}
		Ref<Image> img = Image::create_from_data(n, n, false, Image::FORMAT_R8, px);
		if (cover_tex.is_valid() && cover_tex->get_width() == n && cover_tex->get_height() == n) {
			cover_tex->update(img);
		} else {
			cover_tex = ImageTexture::create_from_image(img);
		}
		cover_px = px;
		cover_gen_built = coverage_generation;
		cover_cam_ei = cam_ei;
		cover_cam_ni = cam_ni;
	}
	const Vector4 rect((real_t)cover_e_min, (real_t)cover_n_min, (real_t)cover_n, 0.0f);
	std::unordered_set<uint64_t> now;
	for (const auto &[key, state] : chunks) {
		if (!state.node) continue;
		const int cg_ei = (int)(int32_t)(key >> 32);
		const int cg_ni = (int)(int32_t)(key & 0xFFFFFFFF);
		const int base_ei = cg_ei * chunk_size, base_ni = cg_ni * chunk_size;
		const int closest_ei = std::max(base_ei, std::min(cam_ei, base_ei + chunk_size - 1));
		const int closest_ni = std::max(base_ni, std::min(cam_ni, base_ni + chunk_size - 1));
		const int min_dist = std::max(std::abs(closest_ei - cam_ei), std::abs(closest_ni - cam_ni));
		if (min_dist > r) continue;
		now.insert(key);
		if (!rebuild && masked_chunks.count(key)) continue;
		Ref<ShaderMaterial> mat = state.node->get_material();
		if (mat.is_null()) continue;
		mat->set_shader_parameter("cover_mask", cover_tex);
		mat->set_shader_parameter("cover_rect", rect);
	}
	for (uint64_t key : masked_chunks) {
		if (now.count(key)) continue;
		auto it = chunks.find(key);
		if (it == chunks.end() || !it->second.node) continue;
		Ref<ShaderMaterial> mat = it->second.node->get_material();
		if (mat.is_valid()) mat->set_shader_parameter("cover_rect", Vector4());
	}
	masked_chunks = std::move(now);
}

bool S3DTileManager::chunk_cover_covers(int p_ei, int p_ni) const
{
	if (cover_n <= 0 || cover_px.size() < (int64_t)cover_n * cover_n) return false;
	const int ie = p_ei - cover_e_min, in = p_ni - cover_n_min;
	if (ie < 0 || ie >= cover_n || in < 0 || in >= cover_n) return false;
	return cover_px[(int64_t)in * cover_n + ie] != 0;
}

Ref<Image> S3DTileManager::landcover_image(const String &p_path, int p_px) const
{
	if (p_px < 1 || p_px > 2048) return Ref<Image>();
	std::vector<uint8_t> rgb;
	std::vector<std::string> cands = { std::string(p_path.utf8().get_data()) };
	if (!s3d_landcover_raster(cands, p_px, (float)tile_size, rgb)) return Ref<Image>();
	PackedByteArray d;
	d.resize((int64_t)rgb.size());
	memcpy(d.ptrw(), rgb.data(), rgb.size());
	return Image::create_from_data(p_px, p_px, false, Image::FORMAT_RGB8, d);
}

void S3DTileManager::set_skip_white_pixels(bool p_skip)
{
	skip_white_pixels = p_skip;
}

bool S3DTileManager::get_skip_white_pixels() const
{
	return skip_white_pixels;
}

void S3DTileManager::set_elevation_db(Ref<S3DElevationDB> p_db)
{
	elevation_db = p_db;
}

godot::Ref<S3DElevationDB> S3DTileManager::get_elevation_db() const
{
	return elevation_db;
}

} // namespace s3d
