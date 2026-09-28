#include <cstddef>

#include "gate.h"

std::vector<double> gate_apply(const std::vector<double> &in, bool relay_open, int cut)
{
	if (relay_open) {
		return std::vector<double>(in.size(), 0.0);
	}
	std::vector<double> out = in;
	if ((cut >= 0) && (static_cast<size_t>(cut) < out.size())) {
		out[static_cast<size_t>(cut)] = 0.0;
	}
	return out;
}
