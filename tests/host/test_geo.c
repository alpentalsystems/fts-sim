#include <math.h>

#include "check.h"
#include "geo.h"

static struct geo_fence square(void)
{
	struct geo_fence f = {.v = {{-100, -100}, {100, -100}, {100, 100}, {-100, 100}},
			      .n = 4, .ceiling_m = 120.0};

	return f;
}

static void test_local_conversion(void)
{
	struct geo_origin o = {.lat_e7 = 375665000, .lon_e7 = 1269780000};
	struct geo_point p;

	geo_to_local(&o, 375665000, 1269780000, &p);
	CHECK(fabs(p.e) < 1e-6 && fabs(p.n) < 1e-6);
	geo_to_local(&o, 375665000 + 8993, 1269780000, &p); /* about 100 m north */
	CHECK(fabs(p.n - 100.0) < 0.5 && fabs(p.e) < 1e-6);
	geo_to_local(&o, 375665000, 1269780000 + 11339, &p); /* about 100 m east at 37.57 N */
	CHECK(fabs(p.e - 100.0) < 0.5 && fabs(p.n) < 1e-6);
}

static void test_inside_outside(void)
{
	struct geo_fence f = square();

	CHECK(geo_inside(&f, (struct geo_point){0, 0}));
	CHECK(geo_inside(&f, (struct geo_point){99.9, -99.9}));
	CHECK(!geo_inside(&f, (struct geo_point){100.1, 0}));
	CHECK(!geo_inside(&f, (struct geo_point){0, -150}));
}

static void test_edges_and_vertices_count_inside(void)
{
	struct geo_fence f = square();

	CHECK(geo_inside(&f, (struct geo_point){100, 0}));
	CHECK(geo_inside(&f, (struct geo_point){0, -100}));
	CHECK(geo_inside(&f, (struct geo_point){100, 100}));
	CHECK(geo_inside(&f, (struct geo_point){-100, -100}));
}

static void test_concave_polygon(void)
{
	/* U shape: the notch (0..50 east, 0..100 north) is outside. */
	struct geo_fence f = {.v = {{-100, -100}, {100, -100}, {100, 100}, {50, 100},
				    {50, 0}, {0, 0}, {0, 100}, {-100, 100}},
			      .n = 8, .ceiling_m = 50.0};

	CHECK(geo_inside(&f, (struct geo_point){-50, 50}));
	CHECK(geo_inside(&f, (struct geo_point){75, 50}));
	CHECK(!geo_inside(&f, (struct geo_point){25, 50}));
	CHECK(geo_inside(&f, (struct geo_point){25, -50}));
}

static void test_fence_validity(void)
{
	struct geo_fence f = square();

	CHECK(geo_fence_valid(&f));
	f.n = 2;
	CHECK(!geo_fence_valid(&f));
	f = square();
	f.ceiling_m = 0.0;
	CHECK(!geo_fence_valid(&f));
	f = square();
	f.n = GEO_MAX_VERTICES + 1;
	CHECK(!geo_fence_valid(&f));
}

int main(void)
{
	test_local_conversion();
	test_inside_outside();
	test_edges_and_vertices_count_inside();
	test_concave_polygon();
	test_fence_validity();
	return CHECK_DONE();
}
