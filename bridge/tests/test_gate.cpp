#include <vector>

#include "check.h"
#include "gate.h"

int main()
{
	const std::vector<double> in = {100.0, 200.0, 300.0, 400.0};
	const std::vector<double> cut = gate_apply(in, false, 2);

	CHECK(gate_apply(in, false, -1) == in);
	CHECK(gate_apply(in, true, -1) == std::vector<double>(4, 0.0));
	CHECK(gate_apply(in, true, 1) == std::vector<double>(4, 0.0));
	CHECK(cut[0] == 100.0 && cut[1] == 200.0 && cut[2] == 0.0 && cut[3] == 400.0);
	return CHECK_DONE();
}
