#include <math.h>

#include "geo.h"

#define EARTH_RADIUS_M 6371000.0
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#define EDGE_EPS_M 1e-6

void geo_to_local(const struct geo_origin *o, int32_t lat_e7, int32_t lon_e7,
		  struct geo_point *p)
{
	double lat0 = (double)o->lat_e7 * 1e-7 * DEG_TO_RAD;
	double dlat = (double)(lat_e7 - o->lat_e7) * 1e-7 * DEG_TO_RAD;
	double dlon = (double)(lon_e7 - o->lon_e7) * 1e-7 * DEG_TO_RAD;

	p->n = dlat * EARTH_RADIUS_M;
	p->e = dlon * EARTH_RADIUS_M * cos(lat0);
}

bool geo_fence_valid(const struct geo_fence *f)
{
	return (f->n >= 3) && (f->n <= GEO_MAX_VERTICES) && (f->ceiling_m > 0.0);
}

static bool on_segment(struct geo_point a, struct geo_point b, struct geo_point p)
{
	double cross = (b.e - a.e) * (p.n - a.n) - (b.n - a.n) * (p.e - a.e);

	if (fabs(cross) > EDGE_EPS_M * (fabs(b.e - a.e) + fabs(b.n - a.n) + 1.0)) {
		return false;
	}
	return (p.e >= fmin(a.e, b.e) - EDGE_EPS_M) && (p.e <= fmax(a.e, b.e) + EDGE_EPS_M) &&
	       (p.n >= fmin(a.n, b.n) - EDGE_EPS_M) && (p.n <= fmax(a.n, b.n) + EDGE_EPS_M);
}

bool geo_inside(const struct geo_fence *f, struct geo_point p)
{
	bool inside = false;

	for (int i = 0, j = f->n - 1; i < f->n; j = i++) {
		struct geo_point a = f->v[i];
		struct geo_point b = f->v[j];

		if (on_segment(a, b, p)) {
			return true;
		}
		if (((a.n > p.n) != (b.n > p.n)) &&
		    (p.e < (b.e - a.e) * (p.n - a.n) / (b.n - a.n) + a.e)) {
			inside = !inside;
		}
	}
	return inside;
}
