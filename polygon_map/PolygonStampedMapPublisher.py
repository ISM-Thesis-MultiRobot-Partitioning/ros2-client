#!/usr/bin/env python3

# import rclpy
from typing import Dict, List, Tuple
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Polygon, Point32


class PolygonStampedMapPublisher(Node):
    def __init__(self, channel: str):
        super().__init__('polygon_stamped_map_publisher')

        self.name = channel
        self.channel = channel
        self.pub = self.create_publisher(PolygonStamped, self.channel, 10)

    def publish(self, points: List[Tuple[Dict[str, float], str]]):
        polygon_msg = PolygonStamped()
        polygon_msg.polygon = Polygon(
            **{'points': [Point32(**p[0]) for p in points]}
        )
        self.pub.publish(polygon_msg)
        self.get_logger().info('Published polygon')
