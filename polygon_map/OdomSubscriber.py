#!/usr/bin/env python3

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from datetime import datetime


class OdomSubscriber(Node):
    def __init__(self, input_channel: str):
        super().__init__(
            'odom_{}_subscriber'.format(input_channel.replace('/', '_'))
        )

        self.channel = input_channel
        self.timeout = 10  # seconds

        self.sub = self.create_subscription(
            Odometry, self.channel, self.callback, 10
        )
        self.value: Optional[Dict[str, float]] = None

    def getOneLocation(self) -> Dict[str, float]:
        self.get_logger().info(f'Get odom location {self.channel}')
        start = datetime.now()
        while rclpy.ok() and not self.value and (datetime.now() - start).total_seconds() < self.timeout:
            rclpy.spin_once(self)
        self.destroy_node()
        value: Dict[str, float] = self.value
        self.value = None
        if value:
            return value
        else:
            self.get_logger().error('Failed to receive position.')
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    def callback(self, msg: Odometry):
        self.value = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
        }
