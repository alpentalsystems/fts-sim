#ifndef BRIDGE_MAVLINK_HB_H_
#define BRIDGE_MAVLINK_HB_H_

#include <cstddef>
#include <cstdint>

/* True if buf holds a MAVLink v2 HEARTBEAT from sysid/compid with a valid CRC. */
bool mav_has_heartbeat(const uint8_t *buf, size_t n, uint8_t sysid, uint8_t compid);

#endif /* BRIDGE_MAVLINK_HB_H_ */
