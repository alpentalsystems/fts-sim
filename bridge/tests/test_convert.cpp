#include "check.h"
#include "convert.h"

int main()
{
	CHECK(stamp_to_us(12, 345678000) == 12345678ULL);
	CHECK(stamp_to_us(0, 999) == 0ULL);
	CHECK(deg_to_e7(47.3979711) == 473979711);
	CHECK(deg_to_e7(-8.54616374) == -85461637);
	CHECK(m_to_mm(488.0) == 488000);
	CHECK(m_to_mm(-1.2345) == -1235);
	return CHECK_DONE();
}
