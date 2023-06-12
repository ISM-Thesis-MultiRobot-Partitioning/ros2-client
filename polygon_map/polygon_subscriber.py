#!/usr/bin/env python3

import rclpy
import os

from .PolygonStampedMapSubscriber import PolygonStampedMapSubscriber

API_URL = os.environ.get('PARTITION_API_URL')
if not API_URL:
    print('PARTITION_API_URL environment variable not set.')
    print('Cannot access the API as a result.')
    print('Exiting.')
    exit(1)


def main(args=None):
    rclpy.init(args=args)

    sub = PolygonStampedMapSubscriber(
        input_channel='/exploration_zone',
        output_channel='/assigned_border',
        api_url=API_URL,
        api_route='PolygonToCellMapContours',
        my_odom_topic='/tb3_1/odom',
        other_odom_topics=[
            # generically get all positions?
            # I.e. do no hard code individual robot's channels?
            '/tb3_0/odom',
            '/tb3_2/odom',
        ],
    )
    rclpy.spin(sub)

    # destroy the node when it is not used anymore
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('Starting ...')
    main()
