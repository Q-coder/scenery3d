#include "s3d_tile_manager.h"
#include "s3d_tile.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>

using namespace godot;

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
	if (load_radius <= 5) {
		lod_rings.push_back({load_radius, 3});
	} else if (load_radius <= 15) {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({load_radius, 4});
	} else if (load_radius <= 50) {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({8, 4});
		lod_rings.push_back({load_radius, 6});
	} else {
		lod_rings.push_back({3, 3});
		lod_rings.push_back({8, 4});
		lod_rings.push_back({25, 6});
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
			int ts = req.tile_size;

			result.composite_res = cres;
			result.tile_size = csize * ts;
			result.desired_lod = 0;

			std::vector<float> composite(cres * cres, NAN);
			bool any_data = false;

			// Cache tile data to avoid re-reading the same file.
			std::unordered_map<uint64_t, std::vector<uint8_t>> tile_cache;
			size_t expected = (size_t)ts * ts * 4;

			for (int cy = 0; cy < cres; cy++) {
				for (int cx = 0; cx < cres; cx++) {
					// Composite pixel (0,0) = SE corner of chunk.
					// cx goes E→W, cy goes S→N (matching provpilot convention).
					double frac_x = (double)cx / (cres - 1); // 0=east edge, 1=west edge
					double frac_y = (double)cy / (cres - 1); // 0=south edge, 1=north edge

					double lv95_e = (double)(base_ei + csize) * ts - frac_x * csize * ts;
					double lv95_n = (double)base_ni * ts + frac_y * csize * ts;

					int tei = (int)std::floor(lv95_e / ts);
					int tni = (int)std::floor(lv95_n / ts);

					// Clamp to chunk bounds.
					if (tei < base_ei) tei = base_ei;
					if (tei >= base_ei + csize) tei = base_ei + csize - 1;
					if (tni < base_ni) tni = base_ni;
					if (tni >= base_ni + csize) tni = base_ni + csize - 1;

					uint64_t tkey = tile_key(tei, tni);
					auto cache_it = tile_cache.find(tkey);
					if (cache_it == tile_cache.end()) {
						int lv95_east = tei * ts;
						int lv95_north = tni * ts;
						std::string tname = "/tile_"
							+ std::to_string(lv95_east) + "_"
							+ std::to_string(lv95_north) + ".raw";
						std::vector<uint8_t> data;
						std::vector<uint8_t> scratch;
						for (const auto &base : req.base_paths) {
							std::ifstream file(base + tname, std::ios::binary);
							if (!file.is_open()) continue;
							scratch.resize(expected);
							file.read(reinterpret_cast<char *>(scratch.data()), expected);
							if ((size_t)file.gcount() != expected) {
								scratch.clear();
								continue;
							}
							if (data.empty()) {
								data = std::move(scratch);
								scratch.clear();
							} else {
								// Merge: fill zero samples from this source.
								float *dst = reinterpret_cast<float *>(data.data());
								const float *src = reinterpret_cast<const float *>(scratch.data());
								size_t n = data.size() / 4;
								for (size_t i = 0; i < n; ++i) {
									if (dst[i] == 0.0f && src[i] != 0.0f) {
										dst[i] = src[i];
									}
								}
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
							int sz = ts;
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
						tile_cache[tkey] = std::move(data);
						cache_it = tile_cache.find(tkey);
					}

					if (cache_it->second.empty()) continue;

					// Block-average sample from the raw tile. Each composite
					// cell covers (ts / (cres-1)) source pixels per axis, so
					// we average a small kx×ky grid within that block rather
					// than picking a single pixel. This is robust against
					// isolated zero / nodata pixels that slip past the
					// per-tile gap-fill above, and dramatically reduces the
					// chance of a whole chunk being classified as empty.
					const float *tile_f = reinterpret_cast<const float *>(cache_it->second.data());
					double local_e = lv95_e - (double)tei * ts;
					double local_n = lv95_n - (double)tni * ts;

					int pcol_c = (int)std::round((ts - local_e) / ts * (ts - 1));
					int prow_c = (int)std::round(local_n / ts * (ts - 1));
					pcol_c = std::max(0, std::min(ts - 1, pcol_c));
					prow_c = std::max(0, std::min(ts - 1, prow_c));

					// Supersampling window: ±K pixels around the nominal
					// sample, clamped to the tile. K ≈ half a composite step
					// (ts / (cres-1) / 2), capped to keep cost bounded.
					int half = std::min(8, (int)(ts / (cres - 1) / 2));

					float sum = 0.0f;
					int cnt = 0;
					for (int dy = -half; dy <= half; dy++) {
						int pr = prow_c + dy;
						if (pr < 0 || pr >= ts) continue;
						for (int dx = -half; dx <= half; dx++) {
							int pc = pcol_c + dx;
							if (pc < 0 || pc >= ts) continue;
							float v = tile_f[(size_t)pr * ts + pc];
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
			if (any_data && !req.ortho_mip_dirs.empty()) {
				result.chunk_tile_count = csize;
				for (int tdy = 0; tdy < csize; tdy++) {
					for (int tdx = 0; tdx < csize; tdx++) {
						int tei = base_ei + tdx;
						int tni = base_ni + tdy;
						int lv95_e = tei * ts;
						int lv95_n = tni * ts;
						std::string fname = "/ortho_"
							+ std::to_string(lv95_e) + "_" + std::to_string(lv95_n) + ".jpg";

						// Try each ortho root in order — first hit wins.
						std::vector<uint8_t> jbuf;
						for (const auto &dir : req.ortho_mip_dirs) {
							std::string opath = dir + fname;
							std::ifstream jpf(opath, std::ios::binary | std::ios::ate);
							if (!jpf.is_open()) continue;
							size_t fsize = (size_t)jpf.tellg();
							if (fsize == 0 || fsize > 256 * 1024) continue;
							jpf.seekg(0, std::ios::beg);
							jbuf.resize(fsize);
							jpf.read(reinterpret_cast<char *>(jbuf.data()), fsize);
							if ((size_t)jpf.gcount() != fsize) { jbuf.clear(); continue; }
							break;
						}
						if (jbuf.empty()) continue;

						int key = tdx * csize + tdy;
						result.chunk_ortho_jpegs[key] = std::move(jbuf);
					}
				}
			}
		} else {
			// --- Regular tile: read single file, trying each candidate path ---
			result.tile_size = req.tile_size;
			result.desired_lod = req.desired_lod;

			size_t expected_size = (size_t)req.tile_size * req.tile_size * 4;
			size_t sample_count = (size_t)req.tile_size * req.tile_size;
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
			std::vector<uint8_t> scratch;
			for (const auto &candidate : req.paths) {
				std::ifstream file(candidate, std::ios::binary);
				if (!file.is_open()) continue;
				scratch.resize(expected_size);
				file.read(reinterpret_cast<char *>(scratch.data()), expected_size);
				if ((size_t)file.gcount() != expected_size) continue;

				if (!result.success) {
					result.raw_bytes = std::move(scratch);
					scratch.clear();
					result.success = true;
				} else {
					float *dst = reinterpret_cast<float *>(result.raw_bytes.data());
					const float *src = reinterpret_cast<const float *>(scratch.data());
					for (size_t i = 0; i < sample_count; ++i) {
						if (src[i] != 0.0f) {
							dst[i] = src[i];
						}
					}
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
				int sz = req.tile_size;
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
						uint8_t *cp = cdata.ptrw();
						const uint8_t *ip = idata.ptr();
						int npx = composite_img->get_width() * composite_img->get_height();
						for (int i = 0; i < npx; i++) {
							int idx = i * 3;
							if (cp[idx] == 255 && cp[idx + 1] == 255 && cp[idx + 2] == 255) {
								cp[idx]     = ip[idx];
								cp[idx + 1] = ip[idx + 1];
								cp[idx + 2] = ip[idx + 2];
							}
						}
						composite_img = Image::create_from_data(
							composite_img->get_width(), composite_img->get_height(),
							false, Image::FORMAT_RGB8, cdata);
					}

					// Quick scan: any white pixels remaining? If not, stop.
					{
						PackedByteArray cdata = composite_img->get_data();
						const uint8_t *cp = cdata.ptr();
						int npx = composite_img->get_width() * composite_img->get_height();
						bool found = false;
						// Sample stride to keep this cheap; full check on small images.
						int step = npx > 4096 ? 16 : 1;
						for (int i = 0; i < npx; i += step) {
							int idx = i * 3;
							if (cp[idx] == 255 && cp[idx + 1] == 255 && cp[idx + 2] == 255) {
								found = true;
								break;
							}
						}
						any_white_left = found;
					}
				}

				if (composite_mode && composite_img.is_valid()) {
					PackedByteArray cdata = composite_img->get_data();
					result.ortho_rgb.assign(cdata.ptr(), cdata.ptr() + cdata.size());
					result.ortho_rgb_w = composite_img->get_width();
					result.ortho_pixels = req.ortho_pixels;
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
				if (!result.chunk_ortho_jpegs.empty() && result.chunk_tile_count > 0) {
					int cs = result.chunk_tile_count;
					int mip_px = 16; // mip3 pixels per tile
					int comp_size = cs * mip_px;
					// Create composite image. Pixel layout matches terrain UV:
					// U=0 at west (high ei), U=1 at east (low ei)
					// V=0 at south (low ni), V=1 at north (high ni)
					Ref<Image> comp_img = Image::create_empty(comp_size, comp_size, false, Image::FORMAT_RGB8);
					bool any_decoded = false;

					for (auto &[tkey, jbuf] : result.chunk_ortho_jpegs) {
						int tdx = tkey / cs; // local x (easting offset)
						int tdy = tkey % cs; // local y (northing offset)

						PackedByteArray jpkg;
						jpkg.resize(jbuf.size());
						memcpy(jpkg.ptrw(), jbuf.data(), jbuf.size());

						Ref<Image> tile_img = Image::create_empty(1, 1, false, Image::FORMAT_RGB8);
						Error err = tile_img->load_jpg_from_buffer(jpkg);
						if (err != OK || tile_img->get_width() < 2) continue;

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

					if (any_decoded) {
						Ref<ImageTexture> tex = ImageTexture::create_from_image(comp_img);
						Ref<StandardMaterial3D> mat;
						mat.instantiate();
						mat->set_texture(StandardMaterial3D::TEXTURE_ALBEDO, tex);
						mat->set_cull_mode(StandardMaterial3D::CULL_DISABLED);
						tile->set_material(mat);
						ortho_applied = true;
					}
				}
				if (!ortho_applied) {
					tile->set_material(shared_material);
				}

				int base_ei = result.ei * chunk_size;
				int base_ni = result.ni * chunk_size;
				double world_x = origin_east - (double)(base_ei + chunk_size) * tile_size;
				double world_z = (double)base_ni * tile_size - origin_north;
				// Sink chunks slightly below individual tiles so where the
				// two overlap the near terrain wins the depth test cleanly.
				// Chunks sample every ~250 m so their surface can otherwise
				// protrude above the detailed tile mesh along ridgelines.
				tile->set_position(Vector3(world_x, CHUNK_Y_BIAS, world_z));

				add_child(tile);
				state.node = tile;
			}

			state.node->set_heightmap(heightmap);
			state.node->set_lod_level(0); // Use every pixel.
			state.node->generate_mesh();
			state.current_lod = 0;
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
			continue;
		}

		int desired_lod = state.desired_lod >= 0 ? state.desired_lod : result.desired_lod;
		int stride = 1 << desired_lod;
		int verts_per_axis = (result.tile_size - 1) / stride + 1;
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

		Ref<Image> heightmap = Image::create_from_data(
			result.tile_size, result.tile_size, false, Image::FORMAT_RF, bytes);
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

			double world_x = origin_east - (double)(result.ei + 1) * tile_size;
			double world_z = (double)result.ni * tile_size - origin_north;
			tile->set_position(Vector3(world_x, 0.0, world_z));

			add_child(tile);
			state.node = tile;
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

		if (desired_lod < LOD_DISCARD_THRESHOLD && elevation_db.is_valid()) {
			elevation_db->load_tile(result.ei, result.ni, heightmap);
		}

		if (desired_lod >= LOD_DISCARD_THRESHOLD) {
			state.node->set_heightmap(Ref<Image>());
		}
	}
}

// --- Bind methods ---

void S3DTileManager::_bind_methods()
{
	ClassDB::bind_method(D_METHOD("update_tiles", "camera_pos"), &S3DTileManager::update_tiles);
	ClassDB::bind_method(D_METHOD("get_active_tile_count"), &S3DTileManager::get_active_tile_count);

	ClassDB::bind_method(D_METHOD("set_tile_size", "size"), &S3DTileManager::set_tile_size);
	ClassDB::bind_method(D_METHOD("get_tile_size"), &S3DTileManager::get_tile_size);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tile_size"), "set_tile_size", "get_tile_size");

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

	ClassDB::bind_method(D_METHOD("set_orthophoto_path", "path"), &S3DTileManager::set_orthophoto_path);
	ClassDB::bind_method(D_METHOD("get_orthophoto_path"), &S3DTileManager::get_orthophoto_path);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "orthophoto_path"), "set_orthophoto_path", "get_orthophoto_path");

	ClassDB::bind_method(D_METHOD("set_orthophoto_paths", "paths"), &S3DTileManager::set_orthophoto_paths);
	ClassDB::bind_method(D_METHOD("get_orthophoto_paths"), &S3DTileManager::get_orthophoto_paths);
	ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "orthophoto_paths"), "set_orthophoto_paths", "get_orthophoto_paths");

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

	// Convert camera world position to LV95 tile indices.
	// +X = West, so LV95_E = origin_east - camera_pos.x
	double cam_lv95_e = origin_east - camera_pos.x;
	double cam_lv95_n = camera_pos.z + origin_north;
	int cam_ei = (int)std::floor(cam_lv95_e / (double)tile_size);
	int cam_ni = (int)std::floor(cam_lv95_n / (double)tile_size);

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
				}
				req.tile_size = tile_size;
				req.desired_lod = desired_lod;
				req.distance = dist;
				req.ortho_paths = ortho_paths_for_tile(ei, ni, desired_lod);
				req.ortho_pixels = req.ortho_paths.empty() ? -1 : ortho_texture_pixels_for_lod(desired_lod);
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
				// Tile is in chunk zone — only unload if chunk is ready.
				int cg_ei = (ei >= 0) ? ei / chunk_size : (ei - chunk_size + 1) / chunk_size;
				int cg_ni = (ni >= 0) ? ni / chunk_size : (ni - chunk_size + 1) / chunk_size;
				uint64_t ckey = tile_key(cg_ei, cg_ni);
				auto cit = chunks.find(ckey);
				if (cit != chunks.end() && !cit->second.loading && cit->second.node) {
					to_remove.push_back(key);
				}
				// else: keep tile until chunk is ready
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
				}
				state.node->queue_free();
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
				int stride = 1 << state.desired_lod;
				int verts_per_axis = (tile_size - 1) / stride + 1;
				int tile_verts = verts_per_axis * verts_per_axis;

				state.node->set_lod_level(state.desired_lod);
				state.node->generate_mesh();

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

				// Discard heightmap if now distant.
				if (state.desired_lod >= LOD_DISCARD_THRESHOLD) {
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
					}
					req.tile_size = tile_size;
					req.desired_lod = state.desired_lod;
					req.distance = 0; // High priority for LOD upgrades.
					req.ortho_paths = ortho_paths_for_tile(ei, ni, state.desired_lod);
					req.ortho_pixels = req.ortho_paths.empty() ? -1 : ortho_texture_pixels_for_lod(state.desired_lod);
					new_requests.push_back(std::move(req));
				}
			}
		}
	}

	// 5. Sort new requests by distance (closest first) and queue to worker.
	if (!new_requests.empty()) {
		std::sort(new_requests.begin(), new_requests.end(),
			[](const LoadRequest &a, const LoadRequest &b) {
				return a.distance < b.distance;
			});

		std::lock_guard<std::mutex> lock(work_mutex);
		for (auto &req : new_requests) {
			work_queue.push_back(std::move(req));
		}
		work_cv.notify_all();
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
			if (it == chunks.end()) {
				TileState state;
				state.desired_lod = 0;
				state.loading = true;
				chunks[ckey] = state;

				LoadRequest req;
				req.is_chunk = true;
				req.ei = cg_ei;
				req.ni = cg_ni;
				req.key = ckey;
				req.chunk_grid_ei = cg_ei;
				req.chunk_grid_ni = cg_ni;
				req.chunk_tile_count = chunk_size;
				req.composite_res = CHUNK_COMPOSITE_RES;
				req.tile_size = tile_size;
				req.base_paths = base_paths_str;
				req.distance = min_dist;
				for (int i = 0; i < orthophoto_paths.size(); i++) {
					String p = orthophoto_paths[i];
					if (p.is_empty()) continue;
					req.ortho_mip_dirs.push_back(std::string(p.utf8().get_data()) + "/mip3");
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
	for (uint64_t key : chunks_to_remove) {
		auto it = chunks.find(key);
		if (it != chunks.end()) {
			if (it->second.node) {
				it->second.node->queue_free();
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

int S3DTileManager::get_active_tile_count() const
{
	return (int)tiles.size() + (int)chunks.size();
}

void S3DTileManager::set_tile_size(int p_size)
{
	tile_size = p_size;
}

int S3DTileManager::get_tile_size() const
{
	return tile_size;
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
}

String S3DTileManager::get_data_path() const
{
	return data_paths.is_empty() ? String() : String(data_paths[0]);
}

void S3DTileManager::set_data_paths(const PackedStringArray &p_paths)
{
	data_paths = p_paths;
}

PackedStringArray S3DTileManager::get_data_paths() const
{
	return data_paths;
}

void S3DTileManager::set_origin_east(double p_east)
{
	origin_east = p_east;
}

double S3DTileManager::get_origin_east() const
{
	return origin_east;
}

void S3DTileManager::set_origin_north(double p_north)
{
	origin_north = p_north;
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

Ref<S3DElevationDB> S3DTileManager::get_elevation_db() const
{
	return elevation_db;
}
