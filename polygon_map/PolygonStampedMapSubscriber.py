#!/usr/bin/env python3

from typing import Dict
from polygon_map.OneTimeSubscriber import OneTimeSubscriber
from polygon_map.PolygonStampedMapPublisher import PolygonStampedMapPublisher

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

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
    ):
        super().__init__('polygon_stamped_map_subscriber')

        self.name = input_channel
        self.input_channel = input_channel
        self.output_channel = output_channel

        self.sub = self.create_subscription(
            PolygonStamped, self.input_channel, self._callback, 10
        )
        self.pub = PolygonStampedMapPublisher(self.output_channel)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)

        self.api_url = api_url
        self.api_route = api_route
        self.map_resolution = {'x': 32, 'y': 32, 'z': 32}

    def getLocation(
        self, target_frame: str = 'kevin', source_frame: str = 'leo02/map'
    ) -> Dict[str, float]:
        try:
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
            vector3 = trans.transform.translation
            pos: Dict[str, float] = {
                'x': vector3.x,
                'y': vector3.y,
                'z': vector3.z,
            }
            return pos
        except Exception as e:
            self.get_logger().error(f'Error getting relative position: {e}')
            raise e

    def getOdomLocation(self, odom_topic: str) -> Dict[str, float]:
        position = OneTimeSubscriber(odom_topic, Odometry).getOnce()
        if position:
            return {
                'x': position.pose.pose.position.x,
                'y': position.pose.pose.position.y,
                'z': position.pose.pose.position.z,
            }
        else:
            return {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
            }

    def _callback(self, data: PolygonStamped):
        print('Data received.')
        start = datetime.now()

        inputdata = {
            'vertices': [
                {'x': p.x, 'y': p.y, 'z': p.z} for p in data.polygon.points
            ],
            'resolution': self.map_resolution,
            'me': self.getOdomLocation('/tb3_1/odom'),
            'others': [
                # generically get all positions?
                # I.e. do no hard code individual robot's channels?
                self.getOdomLocation('/tb3_0/odom'),
                self.getOdomLocation('/tb3_2/odom'),
            ],
        }
        print(
            'Retrieved positions & formulated input data ... ({})'.format(
                datetime.now() - start
            )
        )

        try:
            r = requests.post(f'{self.api_url}/{self.api_route}', json=inputdata)
        except ConnectionError as e:
            self.get_logger().error(f"A connection error occurred. Could not reach the partitioning API: {self.api_url}.\n{e}")
            return
        print('Made query ... ({})'.format(datetime.now() - start))

        if r.status_code != 200:
            print('Error occurred:', r)
            print(r.text)
            return

        jdata = json.loads(r.text)
        print('Parsed JSON ({})'.format(datetime.now() - start))

        print('{} cells were returned.'.format(len(jdata['cells'])))
        print('Offset: {}'.format(len(jdata['offset'])))
        print('Resolution: {}'.format(len(jdata['resolution'])))

        first_few_cells = 10
        print(f'First {first_few_cells} cells:')
        for d in jdata['cells'][:first_few_cells]:
            print(d)

        self.pub.publish(jdata['cells'])
