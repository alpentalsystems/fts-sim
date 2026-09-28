#include <cmath>

#include "convert.h"

uint64_t stamp_to_us(int64_t sec, int32_t nsec)
{
	return static_cast<uint64_t>(sec) * 1000000ULL + static_cast<uint64_t>(nsec / 1000);
}

int32_t deg_to_e7(double deg)
{
	return static_cast<int32_t>(std::llround(deg * 1e7));
}

int32_t m_to_mm(double m)
{
	return static_cast<int32_t>(std::llround(m * 1000.0));
}
