#!/usr/bin/env python3

from typing import Any, Optional

import rclpy
from rclpy.node import Node

from datetime import datetime


class OneTimeSubscriber(Node):
    def __init__(self, input_channel: str, message_type: Any):
        super().__init__(
            'subscriber_{}'.format(input_channel.replace('/', '_'))
        )

        self.channel = input_channel
        self.timeout = 10  # seconds

        self.sub = self.create_subscription(
            message_type, self.channel, self.callback, 10
        )
        self.value: Optional[Any] = None

    def getOnce(self) -> Optional[Any]:
        self.get_logger().info(f'Get odom location {self.channel}')

        start = datetime.now()
        while rclpy.ok() and not self.value and (datetime.now() - start).total_seconds() < self.timeout:
            rclpy.spin_once(self)
        self.destroy_node()

        value: Any = self.value
        self.value = None

        return value

    def callback(self, msg: Any):
        self.value = msg
