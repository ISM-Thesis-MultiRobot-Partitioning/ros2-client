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

MY_ODOM_TOPIC = os.environ.get('MY_ODOM_TOPIC')
if not MY_ODOM_TOPIC:
    print('MY_ODOM_TOPIC environment variable not set.')
    print('Cannot retrieve robot location as a result.')
    print('Exiting.')
    exit(2)

OTHER_ODOM_TOPICS = os.environ.get('OTHER_ODOM_TOPICS')
if not OTHER_ODOM_TOPICS:
    print('OTHER_ODOM_TOPICS environment variable not set.')
    print('Cannot retrieve robot location as a result.')
    print('Exiting.')
    exit(3)
else:
    OTHER_ODOM_TOPICS = OTHER_ODOM_TOPICS.split()


def main(args=None):
    rclpy.init(args=args)

    sub = PolygonStampedMapSubscriber(
        input_channel='/exploration_zone',
        output_channel='/assigned_border',
        api_url=API_URL,
        api_route='PolygonToCellMapContours',
        my_odom_topic=MY_ODOM_TOPIC,
        other_odom_topics=OTHER_ODOM_TOPICS,
    )
    rclpy.spin(sub)

    # destroy the node when it is not used anymore
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print('Starting ...')
    main()
