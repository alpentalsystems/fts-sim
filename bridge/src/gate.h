#ifndef BRIDGE_GATE_H_
#define BRIDGE_GATE_H_

#include <vector>

/* Motor gate: relay open zeroes all motors; cut (0-based, -1 none) zeroes one motor. */
std::vector<double> gate_apply(const std::vector<double> &in, bool relay_open, int cut);

#endif /* BRIDGE_GATE_H_ */
