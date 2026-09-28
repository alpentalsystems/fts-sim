from fts_ground import proto
from fts_ground.link import GroundLink

# STATUS from the C encoder: t_us 12 s, TERMINATED, FENCE (see test_proto.py).
STATUS_FRAME = bytes.fromhex(
    "a50823001bb70000000000030100e079af0000000000e079af0000000000"
    "ffffffffffffffffa287")


def frames(data: bytes) -> list[tuple[int, int, int]]:
    """(type, seq, cmd) for each GS_CMD frame in data."""
    out = []
    for msg_type, payload in proto.Parser().feed(data):
        assert msg_type == proto.MSG_GS_CMD
        out.append((msg_type, int.from_bytes(payload[:4], "little"), payload[4]))
    return out


def test_commands_have_increasing_sequence_numbers():
    link = GroundLink()
    data = link.command(proto.CMD_PING) + link.command(proto.CMD_ARM)
    assert frames(data) == [(proto.MSG_GS_CMD, 1, proto.CMD_PING), (proto.MSG_GS_CMD, 2, proto.CMD_ARM)]


def test_terminate_sends_both_steps_in_order():
    link = GroundLink()
    assert [f[2] for f in frames(link.terminate())] == [proto.CMD_TERMINATE_ARM, proto.CMD_TERMINATE]


def test_disabled_link_sends_nothing():
    link = GroundLink()
    link.enabled = False
    assert link.command(proto.CMD_PING) == b""
    assert link.terminate() == b""
    assert link.t_last_cmd_us is None


def test_status_split_across_reads():
    link = GroundLink()
    assert link.on_bytes(STATUS_FRAME[:10]) == []
    statuses = link.on_bytes(STATUS_FRAME[10:])
    assert len(statuses) == 1 and statuses[0].state == "TERMINATED"
    assert link.last_status == statuses[0]


def test_command_time_is_newest_status_time():
    link = GroundLink()
    link.command(proto.CMD_PING)
    assert link.t_last_cmd_us is None
    link.on_bytes(STATUS_FRAME)
    link.command(proto.CMD_PING)
    assert link.t_last_cmd_us == 12_000_000
