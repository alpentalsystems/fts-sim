#ifndef BRIDGE_CONVERT_H_
#define BRIDGE_CONVERT_H_

#include <cstdint>

uint64_t stamp_to_us(int64_t sec, int32_t nsec);
int32_t deg_to_e7(double deg);
int32_t m_to_mm(double m);

#endif /* BRIDGE_CONVERT_H_ */
