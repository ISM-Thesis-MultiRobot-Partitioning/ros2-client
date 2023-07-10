#!/usr/bin/env python3

from typing import Dict, List, Optional
from polygon_map.OneTimeSubscriber import OneTimeSubscriber
from polygon_map.PolygonStampedMapPublisher import PolygonStampedMapPublisher

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PoseStamped

import requests
from requests.exceptions import ConnectionError
from datetime import datetime
import json


class PolygonStampedMapSubscriber(Node):
    def __init__(
        self,
        input_channel: str,
        output_channel: str,
        api_url: str,
        api_route: str,
        my_odom_topic: str,
        other_odom_topics: List[str],
    ):
        node_name_discriminator = my_odom_topic.replace('/', '_')
        super().__init__(
            f'polygon_stamped_map_subscriber_{node_name_discriminator}'
        )

        self.get_logger().info('Creating Polygon map subscriber ...')

        self.name = input_channel
        self.input_channel = input_channel
        self.output_channel = output_channel

        self.sub = self.create_subscription(
            PolygonStamped, self.input_channel, self._callback, 10
        )
        self.pub = PolygonStampedMapPublisher(
            self.output_channel, node_name_discriminator
        )

        self.api_url = api_url
        self.api_route = api_route
        self.map_resolution = {'x': 32, 'y': 32, 'z': 32}

        self.my_odom_topic: str = my_odom_topic
        self.other_odom_topics: List[str] = other_odom_topics
        self.get_logger().info('Done creating Polygon map subscriber.')

    def _getOdomLocation(self, odom_topic: str) -> Optional[Dict[str, float]]:
        if position := OneTimeSubscriber(odom_topic, PoseStamped).getOnce():
            return {
                'x': position.pose.position.x,
                'y': position.pose.position.y,
                'z': position.pose.position.z,
            }
        else:
            return None

    def getMyOdomLocation(self) -> Optional[Dict[str, float]]:
        return self._getOdomLocation(self.my_odom_topic)

    def getOtherOdomLocations(
        self,
    ) -> List[Dict[str, Optional[Dict[str, float]]]]:
        return [
            {'position': self._getOdomLocation(topic)}
            for topic in self.other_odom_topics
        ]

    def _callback(self, data: PolygonStamped):
        self.get_logger().info('Data received.')
        start = datetime.now()

        inputdata = {
            'vertices': [
                {'x': p.x, 'y': p.y, 'z': p.z} for p in data.polygon.points
            ],
            'resolution': self.map_resolution,
            'me': {'position': self.getMyOdomLocation()},
            'others': self.getOtherOdomLocations(),
        }
        self.get_logger().info(
            'Retrieved positions & formulated input data ... ({})'.format(
                datetime.now() - start
            )
        )

        if not (inputdata['me'] and all(inputdata['others'])):
            self.get_logger().error(
                'One or more topics failed to retrieve odometry information. Aborting.'
            )
            return

        try:
            r = requests.post(
                f'{self.api_url}/{self.api_route}', json=inputdata
            )
        except ConnectionError as e:
            self.get_logger().error(
                'A connection error occurred. Could not reach the partitioning API: {}.\n{}'.format(
                    self.api_url, e
                )
            )
            return
        self.get_logger().info(
            'Made query ... ({})'.format(datetime.now() - start)
        )

        if r.status_code != 200:
            self.get_logger().error(f'Error occurred: {r}\n{r.text}')
            return

        jdata = json.loads(r.text)
        self.get_logger().info(
            'Parsed JSON ({})'.format(datetime.now() - start)
        )

        self.get_logger().info(
            '{} cells were returned.'.format(len(jdata['cells']))
        )
        self.get_logger().info('Offset: {}'.format(len(jdata['offset'])))
        self.get_logger().info(
            'Resolution: {}'.format(len(jdata['resolution']))
        )

        first_few_cells = 10
        self.get_logger().info(f'First {first_few_cells} cells:')
        for d in jdata['cells'][:first_few_cells]:
            self.get_logger().info(d)

        u = []
        [u.append(e) for e in jdata['cells'] if e not in u]
        self.get_logger().info(f'Len: {len(jdata["cells"])}, Uniq: {len(u)}')

        self.pub.publish(jdata['cells'])
