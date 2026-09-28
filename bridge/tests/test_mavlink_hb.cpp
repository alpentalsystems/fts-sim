#include <vector>

#include "check.h"
#include "mavlink_hb.h"

/* HEARTBEAT from sysid 1, compid 1 (pymavlink, MAVLink v2). */
static const uint8_t kHeartbeat[] = {0xfd, 0x09, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
				     0x03, 0x01, 0x00, 0x02, 0x0c, 0x9d, 0x04, 0x03, 0x4e, 0x8f};

int main()
{
	std::vector<uint8_t> bad(kHeartbeat, kHeartbeat + sizeof(kHeartbeat));
	std::vector<uint8_t> noisy = {0x00, 0xfd, 0x01};

	CHECK(mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat), 1, 1));
	CHECK(!mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat), 1, 2));
	CHECK(!mav_has_heartbeat(kHeartbeat, sizeof(kHeartbeat) - 1U, 1, 1));
	bad[12] ^= 0x01U;
	CHECK(!mav_has_heartbeat(bad.data(), bad.size(), 1, 1));
	/* a false start byte before the packet must not hide it */
	noisy.insert(noisy.end(), kHeartbeat, kHeartbeat + sizeof(kHeartbeat));
	CHECK(mav_has_heartbeat(noisy.data(), noisy.size(), 1, 1));
	return CHECK_DONE();
}
