#!/usr/bin/env python3
"""Relay Nav2/teleop TwistStamped on /demo/cmd_vel to plain Twist on /isaac/cmd_vel.

Isaac Sim's ROS2SubscribeTwist OmniGraph node only understands geometry_msgs/Twist,
while the Sambot Nav2 config uses stamped velocity commands (enable_stamped_cmd_vel).
"""
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('input_topic', '/demo/cmd_vel')
        self.declare_parameter('output_topic', '/isaac/cmd_vel')
        self.pub = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(TwistStamped, self.get_parameter('input_topic').value, self.cb, 10)

    def cb(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main():
    rclpy.init()
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
