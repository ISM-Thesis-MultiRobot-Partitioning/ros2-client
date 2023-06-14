#!/usr/bin/env python3

import rclpy
import os

from .PolygonStampedMapSubscriber import PolygonStampedMapSubscriber

API_URL = os.environ.get('PARTITION_API_URL')
MY_ODOM_TOPIC = os.environ.get('MY_ODOM_TOPIC')
OTHER_ODOM_TOPICS = os.environ.get('OTHER_ODOM_TOPICS')
OUTPUT_CHANNEL = os.environ.get('OUTPUT_CHANNEL', '/assigned_border')

if not API_URL:
    print('PARTITION_API_URL environment variable not set.')
    print('Cannot access the API as a result.')
    print('Exiting.')
    exit(1)

if not MY_ODOM_TOPIC:
    print('MY_ODOM_TOPIC environment variable not set.')
    print('Cannot retrieve robot location as a result.')
    print('Exiting.')
    exit(2)

if not OTHER_ODOM_TOPICS:
    print('OTHER_ODOM_TOPICS environment variable not set.')
    print('Cannot retrieve robot locations as a result.')
    print('Exiting.')
    exit(3)
else:
    # topics must be in a space separated string
    OTHER_ODOM_TOPICS = OTHER_ODOM_TOPICS.split()


def main(args=None):
    rclpy.init(args=args)

    sub = PolygonStampedMapSubscriber(
        input_channel='/exploration_zone',
        output_channel=OUTPUT_CHANNEL,
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
