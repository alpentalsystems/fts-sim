"""FTS frame protocol, matching common/include/fts/proto.h."""

import struct
from dataclasses import dataclass

SYNC = 0xA5
MAX_PAYLOAD = 48
OVERHEAD = 5

MSG_IMU = 1
MSG_GNSS = 2
MSG_BARO = 3
MSG_AP_HEARTBEAT = 4
MSG_GS_CMD = 5
MSG_OUTPUT = 6
MSG_READBACK = 7
MSG_STATUS = 8

CMD_PING = 0
CMD_ARM = 1
CMD_DISARM = 2
CMD_TERMINATE_ARM = 3
CMD_TERMINATE = 4

STATES = ("PBIT", "SAFE", "ARMED", "TERMINATED", "FAULT")
CAUSES = ("NONE", "FENCE", "CEILING", "CONTROL", "AP_FREEZE", "LINK", "MANUAL", "GNSS_LOST")

UNSET_US = 0xFFFFFFFFFFFFFFFF

_STATUS = struct.Struct("<QBBBQQQ")
STATUS_LEN = _STATUS.size


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_type: int, payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)} bytes")
    body = bytes((msg_type, len(payload))) + payload
    return bytes((SYNC,)) + body + struct.pack("<H", crc16(body))


def encode_imu(t_us: int, q, gyro, accel) -> bytes:
    return encode(MSG_IMU, struct.pack("<Q10f", t_us, *q, *gyro, *accel))


def encode_gnss(t_us: int, lat_e7: int, lon_e7: int, alt_mm: int, fix: int, sats: int) -> bytes:
    return encode(MSG_GNSS, struct.pack("<QiiiBB", t_us, lat_e7, lon_e7, alt_mm, fix, sats))


def encode_baro(t_us: int, pressure_pa: float, temp_c: float) -> bytes:
    return encode(MSG_BARO, struct.pack("<Qff", t_us, pressure_pa, temp_c))


def encode_ap_heartbeat(t_us: int, seq: int) -> bytes:
    return encode(MSG_AP_HEARTBEAT, struct.pack("<QI", t_us, seq))


def encode_gs_cmd(seq: int, cmd: int) -> bytes:
    return encode(MSG_GS_CMD, struct.pack("<IB", seq, cmd))


def encode_readback(t_us: int, relay_open: int, chute_fired: int) -> bytes:
    return encode(MSG_READBACK, struct.pack("<QBB", t_us, relay_open, chute_fired))


@dataclass(frozen=True)
class Status:
    t_us: int
    state: str
    cause: str
    pbit_fail: int
    t_trigger_us: int | None
    t_relay_us: int | None
    t_chute_us: int | None


def _time(value: int) -> int | None:
    return None if value == UNSET_US else value


def decode_status(payload: bytes) -> Status:
    t_us, state, cause, pbit_fail, t_trigger, t_relay, t_chute = _STATUS.unpack(payload)
    return Status(t_us, STATES[state], CAUSES[cause], pbit_fail,
                  _time(t_trigger), _time(t_relay), _time(t_chute))


@dataclass(frozen=True)
class Output:
    relay_open: bool
    chute_fire: bool


def decode_output(payload: bytes) -> Output:
    relay_open, chute_fire = struct.unpack("<BB", payload)
    return Output(relay_open != 0, chute_fire != 0)


class Parser:
    """Byte stream to frames. A bad candidate drops only its sync byte."""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.crc_errors = 0
        self.len_errors = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buf += data
        frames = []
        while True:
            start = self.buf.find(SYNC)
            if start < 0:
                self.buf.clear()
                return frames
            del self.buf[:start]
            if len(self.buf) < 3:
                return frames
            length = self.buf[2]
            if length > MAX_PAYLOAD:
                self.len_errors += 1
                del self.buf[:1]
                continue
            total = OVERHEAD + length
            if len(self.buf) < total:
                return frames
            body = bytes(self.buf[1:3 + length])
            (rx_crc,) = struct.unpack_from("<H", self.buf, 3 + length)
            if crc16(body) != rx_crc:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            frames.append((body[0], body[2:]))
            del self.buf[:total]
