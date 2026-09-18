// s3d_proj.h — rigorous map projections shared by Scenery3D and its hosts.
//
// Header-only, no Godot dependencies, so the same code can be included from
// the ProVPilot extension (build adds this directory to CPPPATH) and from
// tests. Everything is in double precision; angles in DEGREES at the API,
// radians internally.
//
// Provided:
//   * WGS84 <-> Swiss LV95 / CH1903+ (EPSG:2056): the swisstopo "rigorous"
//     Swiss conformal cylindrical (oblique Mercator) formulas on the Bessel
//     1841 ellipsoid plus the 3-parameter geocentric datum shift CH1903+ <->
//     ETRS89/WGS84 (dX +674.374, dY +15.056, dZ +405.346 m). This is exactly
//     the pipeline pyproj/PROJ uses for EPSG:4326 <-> EPSG:2056 without the
//     CHENyx06 grid, i.e. what ProVPilot's Python bake scripts produced —
//     the runtime now matches them to the millimetre anywhere in Europe.
//     The older swisstopo *approximation polynomial* (~1 m inside CH) drifts
//     by kilometres outside it (24 km at 59°N, 207 km at 71°N) and must not
//     be used for anything beyond Switzerland.
//   * Transverse Mercator / UTM (Krüger series to n^4, sub-mm inside a zone)
//     for regions that declare a UTM CRS (design §9, D6).
//   * Grid convergence (bearing of true north in grid terms) so true runway
//     bearings can be placed in a grid-north frame.
//
// References: swisstopo, "Formulas and constants for the calculation of the
// Swiss conformal cylindrical projection and for the transformation between
// coordinate systems" (2016); Karney, "Transverse Mercator with an accuracy
// of a few nanometers" (2011).
#ifndef S3D_PROJ_H
#define S3D_PROJ_H

#include <cmath>

namespace s3d
{
namespace proj
{

	struct LatLon { double lat = 0.0, lon = 0.0; };   // degrees
	struct EN { double e = 0.0, n = 0.0; };           // metres
	struct XYZ { double x = 0.0, y = 0.0, z = 0.0; }; // metres (ECEF)

	struct Ellipsoid
	{
		double a; // semi-major axis
		double f; // flattening
		double e2() const { return f * (2.0 - f); }
	};
	constexpr Ellipsoid WGS84{6378137.0, 1.0 / 298.257223563};
	constexpr Ellipsoid BESSEL1841{6377397.155, 1.0 / 299.1528128};

	constexpr double PI = 3.14159265358979323846;
	inline double deg2rad(double d) { return d * PI / 180.0; }
	inline double rad2deg(double r) { return r * 180.0 / PI; }

	// ---- geodetic <-> ECEF on an arbitrary ellipsoid ----

	inline XYZ geodetic_to_ecef(const Ellipsoid &ell, double lat_deg, double lon_deg, double h)
	{
		const double lat = deg2rad(lat_deg), lon = deg2rad(lon_deg);
		const double s = std::sin(lat), c = std::cos(lat);
		const double N = ell.a / std::sqrt(1.0 - ell.e2() * s * s);
		return XYZ{(N + h) * c * std::cos(lon), (N + h) * c * std::sin(lon), (N * (1.0 - ell.e2()) + h) * s};
	}

	// Returns lat/lon in degrees; *h_out (optional) gets the ellipsoidal height.
	inline LatLon ecef_to_geodetic(const Ellipsoid &ell, const XYZ &p, double *h_out = nullptr)
	{
		const double e2 = ell.e2();
		const double lon = std::atan2(p.y, p.x);
		const double r = std::hypot(p.x, p.y);
		double lat = std::atan2(p.z, r * (1.0 - e2)); // initial guess
		double N = ell.a, h = 0.0;
		for (int i = 0; i < 8; ++i)
		{
			const double s = std::sin(lat);
			N = ell.a / std::sqrt(1.0 - e2 * s * s);
			h = r / std::cos(lat) - N;
			const double next = std::atan2(p.z, r * (1.0 - e2 * N / (N + h)));
			if (std::fabs(next - lat) < 1e-14)
			{
				lat = next;
				break;
			}
			lat = next;
		}
		if (h_out)
		{
			*h_out = h;
		}
		return LatLon{rad2deg(lat), rad2deg(lon)};
	}

	// ---- datum shift CH1903+ <-> WGS84 (EPSG:1676, geocentric translation) ----

	constexpr double CH_DX = 674.374, CH_DY = 15.056, CH_DZ = 405.346;

	inline LatLon wgs84_to_ch1903plus(double lat_deg, double lon_deg)
	{
		XYZ p = geodetic_to_ecef(WGS84, lat_deg, lon_deg, 0.0);
		p.x -= CH_DX; p.y -= CH_DY; p.z -= CH_DZ;
		return ecef_to_geodetic(BESSEL1841, p);
	}

	inline LatLon ch1903plus_to_wgs84(double lat_deg, double lon_deg)
	{
		XYZ p = geodetic_to_ecef(BESSEL1841, lat_deg, lon_deg, 0.0);
		p.x += CH_DX; p.y += CH_DY; p.z += CH_DZ;
		return ecef_to_geodetic(WGS84, p);
	}

	// ---- Swiss oblique Mercator (rigorous), CH1903+ Bessel geodetic <-> LV95 ----

	namespace swiss
	{
		constexpr double A = 6377397.155;
		constexpr double E2 = 0.006674372230614;
		constexpr double LAT0 = 46.0 + 57.0 / 60.0 + 8.66 / 3600.0;   // 46°57'08.66"
		constexpr double LON0 = 7.0 + 26.0 / 60.0 + 22.50 / 3600.0;   // 7°26'22.50"
		constexpr double E0 = 2600000.0, N0 = 1200000.0;

		struct Consts
		{
			double e, r, alpha, b0, k;
			Consts()
			{
				const double phi0 = deg2rad(LAT0);
				e = std::sqrt(E2);
				const double s0 = std::sin(phi0), c0 = std::cos(phi0);
				r = A * std::sqrt(1.0 - E2) / (1.0 - E2 * s0 * s0);
				alpha = std::sqrt(1.0 + E2 / (1.0 - E2) * c0 * c0 * c0 * c0);
				b0 = std::asin(s0 / alpha);
				k = std::log(std::tan(PI / 4.0 + b0 / 2.0)) - alpha * std::log(std::tan(PI / 4.0 + phi0 / 2.0)) + alpha * e / 2.0 * std::log((1.0 + e * s0) / (1.0 - e * s0));
			}
		};
		inline const Consts &consts()
		{
			static const Consts c;
			return c;
		}
	}

	// CH1903+ (Bessel) geodetic -> LV95 E/N.
	inline EN ch1903plus_to_lv95(double lat_deg, double lon_deg)
	{
		const swiss::Consts &c = swiss::consts();
		const double phi = deg2rad(lat_deg), lam = deg2rad(lon_deg);
		const double sp = std::sin(phi);
		const double S = c.alpha * std::log(std::tan(PI / 4.0 + phi / 2.0)) - c.alpha * c.e / 2.0 * std::log((1.0 + c.e * sp) / (1.0 - c.e * sp)) + c.k;
		const double b = 2.0 * (std::atan(std::exp(S)) - PI / 4.0);
		const double l = c.alpha * (lam - deg2rad(swiss::LON0));
		const double sb0 = std::sin(c.b0), cb0 = std::cos(c.b0);
		const double lbar = std::atan2(std::sin(l), sb0 * std::tan(b) + cb0 * std::cos(l));
		const double bbar = std::asin(cb0 * std::sin(b) - sb0 * std::cos(b) * std::cos(l));
		const double Y = c.r * lbar;
		const double X = c.r / 2.0 * std::log((1.0 + std::sin(bbar)) / (1.0 - std::sin(bbar)));
		return EN{Y + swiss::E0, X + swiss::N0};
	}

	// LV95 E/N -> CH1903+ (Bessel) geodetic.
	inline LatLon lv95_to_ch1903plus(double e, double n)
	{
		const swiss::Consts &c = swiss::consts();
		const double Y = e - swiss::E0, X = n - swiss::N0;
		const double lbar = Y / c.r;
		const double bbar = 2.0 * (std::atan(std::exp(X / c.r)) - PI / 4.0);
		const double sb0 = std::sin(c.b0), cb0 = std::cos(c.b0);
		const double b = std::asin(cb0 * std::sin(bbar) + sb0 * std::cos(bbar) * std::cos(lbar));
		const double l = std::atan2(std::sin(lbar), cb0 * std::cos(lbar) - sb0 * std::tan(bbar));
		const double lam = deg2rad(swiss::LON0) + l / c.alpha;
		const double S = (std::log(std::tan(PI / 4.0 + b / 2.0)) - c.k) / c.alpha;
		double phi = b;
		for (int i = 0; i < 10; ++i)
		{
			const double sp = std::sin(phi);
			const double next = 2.0 * std::atan(std::exp(S + c.e / 2.0 * std::log((1.0 + c.e * sp) / (1.0 - c.e * sp)))) - PI / 2.0;
			if (std::fabs(next - phi) < 1e-14)
			{
				phi = next;
				break;
			}
			phi = next;
		}
		return LatLon{rad2deg(phi), rad2deg(lam)};
	}

	// WGS84 <-> LV95 (EPSG:4326 <-> EPSG:2056), the pair hosts should use.
	inline EN wgs84_to_lv95(double lat_deg, double lon_deg)
	{
		const LatLon ch = wgs84_to_ch1903plus(lat_deg, lon_deg);
		return ch1903plus_to_lv95(ch.lat, ch.lon);
	}

	inline LatLon lv95_to_wgs84(double e, double n)
	{
		const LatLon ch = lv95_to_ch1903plus(e, n);
		return ch1903plus_to_wgs84(ch.lat, ch.lon);
	}

	// Grid convergence of LV95 at a WGS84 point: bearing of TRUE north measured
	// clockwise from LV95 grid north, in degrees. A true bearing T becomes the
	// grid bearing T + convergence. (Numerical: project a short step north.)
	inline double lv95_convergence_deg(double lat_deg, double lon_deg)
	{
		const double d = 1e-4; // ~11 m
		const EN p0 = wgs84_to_lv95(lat_deg - d, lon_deg);
		const EN p1 = wgs84_to_lv95(lat_deg + d, lon_deg);
		return rad2deg(std::atan2(p1.e - p0.e, p1.n - p0.n));
	}

	// ---- Transverse Mercator (Krüger series) and UTM ----

	struct TMParams
	{
		Ellipsoid ell = WGS84;
		double lon0_deg = 0.0;
		double k0 = 0.9996;
		double false_e = 500000.0;
		double false_n = 0.0;
	};

	namespace tm_detail
	{
		struct Series
		{
			double n, A, alpha[4], beta[4], delta[4];
			explicit Series(const Ellipsoid &ell)
			{
				n = ell.f / (2.0 - ell.f);
				const double n2 = n * n, n3 = n2 * n, n4 = n3 * n;
				A = ell.a / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0);
				alpha[0] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0;
				alpha[1] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0;
				alpha[2] = 61.0 * n3 / 240.0 - 103.0 * n4 / 140.0;
				alpha[3] = 49561.0 * n4 / 161280.0;
				beta[0] = n / 2.0 - 2.0 * n2 / 3.0 + 37.0 * n3 / 96.0 - n4 / 360.0;
				beta[1] = n2 / 48.0 + n3 / 15.0 - 437.0 * n4 / 1440.0;
				beta[2] = 17.0 * n3 / 480.0 - 37.0 * n4 / 840.0;
				beta[3] = 4397.0 * n4 / 161280.0;
				delta[0] = 2.0 * n - 2.0 * n2 / 3.0 - 2.0 * n3 + 116.0 * n4 / 45.0;
				delta[1] = 7.0 * n2 / 3.0 - 8.0 * n3 / 5.0 - 227.0 * n4 / 45.0;
				delta[2] = 56.0 * n3 / 15.0 - 136.0 * n4 / 35.0;
				delta[3] = 4279.0 * n4 / 630.0;
			}
		};
	}

	inline EN tm_forward(const TMParams &p, double lat_deg, double lon_deg)
	{
		const tm_detail::Series s(p.ell);
		const double phi = deg2rad(lat_deg);
		const double dl = deg2rad(lon_deg - p.lon0_deg);
		const double q = 2.0 * std::sqrt(s.n) / (1.0 + s.n);
		const double t = std::sinh(std::atanh(std::sin(phi)) - q * std::atanh(q * std::sin(phi)));
		const double xi_p = std::atan2(t, std::cos(dl));
		const double eta_p = std::atanh(std::sin(dl) / std::sqrt(1.0 + t * t));
		double xi = xi_p, eta = eta_p;
		for (int j = 1; j <= 4; ++j)
		{
			xi += s.alpha[j - 1] * std::sin(2.0 * j * xi_p) * std::cosh(2.0 * j * eta_p);
			eta += s.alpha[j - 1] * std::cos(2.0 * j * xi_p) * std::sinh(2.0 * j * eta_p);
		}
		return EN{p.k0 * s.A * eta + p.false_e, p.k0 * s.A * xi + p.false_n};
	}

	inline LatLon tm_inverse(const TMParams &p, double e, double n)
	{
		const tm_detail::Series s(p.ell);
		const double xi = (n - p.false_n) / (p.k0 * s.A);
		const double eta = (e - p.false_e) / (p.k0 * s.A);
		double xi_p = xi, eta_p = eta;
		for (int j = 1; j <= 4; ++j)
		{
			xi_p -= s.beta[j - 1] * std::sin(2.0 * j * xi) * std::cosh(2.0 * j * eta);
			eta_p -= s.beta[j - 1] * std::cos(2.0 * j * xi) * std::sinh(2.0 * j * eta);
		}
		const double chi = std::asin(std::sin(xi_p) / std::cosh(eta_p));
		double phi = chi;
		for (int j = 1; j <= 4; ++j)
		{
			phi += s.delta[j - 1] * std::sin(2.0 * j * chi);
		}
		const double lam = deg2rad(p.lon0_deg) + std::atan2(std::sinh(eta_p), std::cos(xi_p));
		return LatLon{rad2deg(phi), rad2deg(lam)};
	}

	inline TMParams utm_params(int zone, bool north)
	{
		TMParams p;
		p.lon0_deg = zone * 6.0 - 183.0;
		p.false_n = north ? 0.0 : 10000000.0;
		return p;
	}
	inline EN wgs84_to_utm(double lat_deg, double lon_deg, int zone, bool north = true)
	{
		return tm_forward(utm_params(zone, north), lat_deg, lon_deg);
	}
	inline LatLon utm_to_wgs84(double e, double n, int zone, bool north = true)
	{
		return tm_inverse(utm_params(zone, north), e, n);
	}
	inline int utm_zone_for(double lon_deg)
	{
		int z = (int)std::floor((lon_deg + 180.0) / 6.0) + 1;
		return z < 1 ? 1 : (z > 60 ? 60 : z);
	}

	// ---- Region CRS (design §9 D6): which projection a scenery pack's tile
	// grid is expressed in. LV95 covers CH/BW and the Fennoscandia bakes; UTM
	// is for any future region far from Switzerland. ----
	struct Crs
	{
		enum Kind { LV95, UTM };
		Kind kind = LV95;
		int utm_zone = 33;
		bool utm_north = true;

		EN forward(double lat_deg, double lon_deg) const
		{
			return kind == LV95 ? wgs84_to_lv95(lat_deg, lon_deg) : wgs84_to_utm(lat_deg, lon_deg, utm_zone, utm_north);
		}
		LatLon inverse(double e, double n) const
		{
			return kind == LV95 ? lv95_to_wgs84(e, n) : utm_to_wgs84(e, n, utm_zone, utm_north);
		}
		// Bearing of true north clockwise from grid north (degrees).
		double convergence_deg(double lat_deg, double lon_deg) const
		{
			const double d = 1e-4;
			const EN p0 = forward(lat_deg - d, lon_deg);
			const EN p1 = forward(lat_deg + d, lon_deg);
			return rad2deg(std::atan2(p1.e - p0.e, p1.n - p0.n));
		}
		// Parse "EPSG:2056" / "EPSG:326xx" / "EPSG:327xx" / "EPSG:258xx"; false if unknown.
		static bool from_epsg(int code, Crs &out)
		{
			if (code == 2056) { out.kind = LV95; return true; }
			if (code >= 32601 && code <= 32660) { out.kind = UTM; out.utm_zone = code - 32600; out.utm_north = true; return true; }
			if (code >= 32701 && code <= 32760) { out.kind = UTM; out.utm_zone = code - 32700; out.utm_north = false; return true; }
			if (code >= 25828 && code <= 25838) { out.kind = UTM; out.utm_zone = code - 25800; out.utm_north = true; return true; } // ETRS89 / UTM (datum diff < 1 m, ignored)
			return false;
		}
	};

} // namespace proj
} // namespace s3d

#endif // S3D_PROJ_H
