#!/usr/bin/env python3
"""Publish empty JointState so robot_state_publisher publishes the TF tree."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class StaticJointPublisher(Node):
    def __init__(self):
        super().__init__('field_joint_state_pub')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(StaticJointPublisher())


if __name__ == '__main__':
    main()
