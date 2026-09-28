#include "mavlink_hb.h"

namespace {

constexpr uint8_t kStx = 0xFD;
constexpr size_t kHeader = 10;   /* stx, len, incompat, compat, seq, sysid, compid, msgid x3 */
constexpr size_t kSignature = 13;
constexpr uint8_t kSigned = 0x01;
constexpr uint8_t kHeartbeatCrcExtra = 50;

uint16_t x25(uint16_t crc, uint8_t b)
{
	uint8_t t = static_cast<uint8_t>(b ^ static_cast<uint8_t>(crc & 0xFFU));

	t = static_cast<uint8_t>(t ^ static_cast<uint8_t>(t << 4));
	return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(t) << 8) ^
				     (static_cast<uint16_t>(t) << 3) ^ (t >> 4));
}

} // namespace

bool mav_has_heartbeat(const uint8_t *buf, size_t n, uint8_t sysid, uint8_t compid)
{
	for (size_t i = 0; i + kHeader + 2U <= n; i++) {
		if (buf[i] != kStx) {
			continue;
		}
		const size_t len = buf[i + 1];
		const size_t total = kHeader + len + 2U + (((buf[i + 2] & kSigned) != 0U) ? kSignature : 0U);
		const uint32_t msgid = buf[i + 7] | (buf[i + 8] << 8) | (static_cast<uint32_t>(buf[i + 9]) << 16);

		if ((i + total > n) || (msgid != 0U) || (buf[i + 5] != sysid) || (buf[i + 6] != compid)) {
			continue;
		}
		uint16_t crc = 0xFFFF;
		for (size_t k = i + 1; k < i + kHeader + len; k++) {
			crc = x25(crc, buf[k]);
		}
		crc = x25(crc, kHeartbeatCrcExtra);
		const uint16_t rx = static_cast<uint16_t>(buf[i + kHeader + len] | (buf[i + kHeader + len + 1] << 8));
		if (crc == rx) {
			return true;
		}
	}
	return false;
}
