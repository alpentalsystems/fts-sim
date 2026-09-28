"""FTS ground station: link 2 over a PTY, as a ROS 2 topic and services."""

import os
import tty

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from fts_ground import proto
from fts_ground.link import GroundLink
from fts_interfaces.msg import FtsStatus

PING_PERIOD_S = 0.5
POLL_PERIOD_S = 0.01


def _us(value: int | None) -> int:
    return -1 if value is None else value


class GroundStation(Node):
    def __init__(self) -> None:
        super().__init__("fts_ground_station")
        path = self.declare_parameter("link", "").get_parameter_value().string_value
        if not path:
            raise RuntimeError("parameter 'link' (PTY path) is required")
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        self.link = GroundLink()
        self.state = ""
        self.pub = self.create_publisher(FtsStatus, "fts/status", 10)
        self.create_timer(POLL_PERIOD_S, self.poll)
        self.create_timer(PING_PERIOD_S, lambda: self.write(self.link.command(proto.CMD_PING)))
        self.create_service(Trigger, "~/arm", self.command_service(proto.CMD_ARM))
        self.create_service(Trigger, "~/disarm", self.command_service(proto.CMD_DISARM))
        self.create_service(Trigger, "~/terminate", self.on_terminate)
        self.create_service(SetBool, "~/link", self.on_link)
        self.get_logger().info(f"link 2 on {path}")

    def write(self, data: bytes) -> None:
        if data:
            os.write(self.fd, data)

    def reply(self, resp):
        resp.success = self.link.enabled
        resp.message = "sent" if self.link.enabled else "link disabled"
        return resp

    def command_service(self, cmd: int):
        def handle(_req, resp):
            self.write(self.link.command(cmd))
            return self.reply(resp)
        return handle

    def on_terminate(self, _req, resp):
        self.write(self.link.terminate())
        return self.reply(resp)

    def on_link(self, req, resp):
        self.link.enabled = req.data
        resp.success = True
        resp.message = "link enabled" if req.data else "link disabled"
        self.get_logger().info(resp.message)
        return resp

    def poll(self) -> None:
        while True:
            try:
                data = os.read(self.fd, 4096)
            except BlockingIOError:
                return
            if not data:
                raise RuntimeError("link 2 closed")
            for st in self.link.on_bytes(data):
                if st.state != self.state:
                    self.get_logger().info(f"FTS {st.state} cause {st.cause}")
                    self.state = st.state
                self.pub.publish(self.to_msg(st))

    def to_msg(self, st: proto.Status) -> FtsStatus:
        msg = FtsStatus()
        msg.t_us = st.t_us
        msg.state = st.state
        msg.cause = st.cause
        msg.pbit_fail = st.pbit_fail
        msg.t_trigger_us = _us(st.t_trigger_us)
        msg.t_relay_us = _us(st.t_relay_us)
        msg.t_chute_us = _us(st.t_chute_us)
        msg.t_last_cmd_us = _us(self.link.t_last_cmd_us)
        msg.link_enabled = self.link.enabled
        return msg


def main() -> None:
    rclpy.init()
    node = GroundStation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
