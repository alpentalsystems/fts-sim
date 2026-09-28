#include "check.h"

int main(void)
{
	CHECK(1 + 1 == 2);
	return CHECK_DONE();
}
