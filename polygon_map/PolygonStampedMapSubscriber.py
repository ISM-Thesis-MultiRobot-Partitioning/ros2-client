#!/usr/bin/env python3

from typing import Dict, List
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from tf2_ros import Buffer, TransformListener


class PolygonStampedMapSubscriber(Node):

    def __init__(self, channel):
        super().__init__('polygon_stamped_map_subscriber')

        self.name = channel
        self.channel = channel
        self.sub = self.create_subscription(
            PolygonStamped,
            self.channel,
            self._callback,
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

    def getLocation(self,
                    target_frame: str = 'kevin',
                    source_frame: str = 'leo02/map'
                    ) -> Dict[str, float]:
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            pos: Dict[str, float] = trans.transform.translation
            return pos
        except Exception as e:
            self.get_logger().error(f'Error getting relative position: {e}')
            raise e

    def _callback(self, data: PolygonStamped):
        print("Data received:", data)
        return
        myloc: Dict[str, float] = self.getLocation('kevin', 'leo02/map')
        other: List[Dict[str, float]] = [
            # generically get all positions?
            # I.e. do no hard code individual robot's channels?
            self.getLocation('leo02/base_link', 'leo02/map'),
        ]
        # TODO: forward information to API
        # TODO: obtain results and so something with them?
        pass
