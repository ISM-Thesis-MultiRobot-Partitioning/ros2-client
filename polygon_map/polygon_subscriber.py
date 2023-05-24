#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

from .PolygonStampedMapSubscriber import PolygonStampedMapSubscriber


def main(args=None):
    rclpy.init(args=args)

    sub = PolygonStampedMapSubscriber('/exploration_zone')
    rclpy.spin(sub)

    # destroy the node when it is not used anymore
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
