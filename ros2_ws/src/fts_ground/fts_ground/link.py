"""Ground station side of link 2, without ROS."""

from fts_ground import proto


class GroundLink:
    def __init__(self) -> None:
        self.parser = proto.Parser()
        self.seq = 0
        self.enabled = True
        self.last_status: proto.Status | None = None
        self.t_last_cmd_us: int | None = None

    def command(self, cmd: int) -> bytes:
        """Frame for one command, or nothing while the link is disabled."""
        if not self.enabled:
            return b""
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        if self.last_status is not None:
            self.t_last_cmd_us = self.last_status.t_us
        return proto.encode_gs_cmd(self.seq, cmd)

    def terminate(self) -> bytes:
        return self.command(proto.CMD_TERMINATE_ARM) + self.command(proto.CMD_TERMINATE)

    def on_bytes(self, data: bytes) -> list[proto.Status]:
        statuses = []
        for msg_type, payload in self.parser.feed(data):
            if msg_type == proto.MSG_STATUS and len(payload) == proto.STATUS_LEN:
                self.last_status = proto.decode_status(payload)
                statuses.append(self.last_status)
        return statuses
