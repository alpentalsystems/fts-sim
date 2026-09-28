import pytest

from fts_ground import proto

# Produced by common/proto.c.
GS_CMD_FRAME = bytes.fromhex("a5050509000000049023")
STATUS_FRAME = bytes.fromhex(
    "a50823001bb70000000000030100e079af0000000000e079af0000000000"
    "ffffffffffffffffa287")
IMU_FRAME = bytes.fromhex(
    "a50130e8030000000000000000803f000000000000000000000000000000"
    "00000000000000003f0000000000000000c3f51c418f44")


def test_crc16_check_value():
    assert proto.crc16(b"123456789") == 0x29B1


def test_gs_cmd_matches_c_encoder():
    assert proto.encode_gs_cmd(9, proto.CMD_TERMINATE) == GS_CMD_FRAME


def test_imu_matches_c_encoder():
    frame = proto.encode_imu(1000, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.5), (0.0, 0.0, 9.81))
    assert frame == IMU_FRAME


def test_status_from_c_encoder():
    frames = proto.Parser().feed(STATUS_FRAME)
    assert len(frames) == 1
    msg_type, payload = frames[0]
    assert msg_type == proto.MSG_STATUS
    assert proto.decode_status(payload) == proto.Status(
        t_us=12_000_000, state="TERMINATED", cause="FENCE", pbit_fail=0,
        t_trigger_us=11_500_000, t_relay_us=11_500_000, t_chute_us=None)


def test_output_decode():
    assert proto.decode_output(bytes((1, 0))) == proto.Output(relay_open=True, chute_fire=False)


def test_parser_handles_split_noise_and_bad_crc():
    bad = bytearray(GS_CMD_FRAME)
    bad[-1] ^= 0xFF
    data = b"\x00\xa5\x07" + bytes(bad) + GS_CMD_FRAME
    parser = proto.Parser()
    frames = parser.feed(data[:7]) + parser.feed(data[7:])
    assert frames == [(proto.MSG_GS_CMD, GS_CMD_FRAME[3:-2])]
    assert parser.crc_errors >= 1


def test_encode_rejects_long_payload():
    with pytest.raises(ValueError):
        proto.encode(proto.MSG_IMU, bytes(proto.MAX_PAYLOAD + 1))
