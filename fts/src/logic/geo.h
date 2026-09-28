#ifndef GEO_H_
#define GEO_H_

#include <stdbool.h>
#include <stdint.h>

#define GEO_MAX_VERTICES 16

struct geo_origin {
	int32_t lat_e7;
	int32_t lon_e7;
};

/* Meters east and north of the origin. */
struct geo_point {
	double e;
	double n;
};

struct geo_fence {
	struct geo_point v[GEO_MAX_VERTICES];
	int n;
	double ceiling_m; /* above the origin */
};

/* Local flat-earth conversion; accurate to well under 1 m within a few km. */
void geo_to_local(const struct geo_origin *o, int32_t lat_e7, int32_t lon_e7,
		  struct geo_point *p);

/* At least 3 vertices, at most GEO_MAX_VERTICES, ceiling above ground. */
bool geo_fence_valid(const struct geo_fence *f);

/* Points on an edge or vertex count as inside. */
bool geo_inside(const struct geo_fence *f, struct geo_point p);

#endif /* GEO_H_ */
