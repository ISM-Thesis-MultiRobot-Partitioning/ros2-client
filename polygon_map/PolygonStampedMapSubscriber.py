#!/usr/bin/env python3

from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from tf2_ros import Buffer, TransformListener

import requests
from datetime import datetime
import json


class PolygonStampedMapSubscriber(Node):

    def __init__(self, channel: str, api_url: str):
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

        self.api_url = api_url
        self.api_route = "PolygonToCellMap"
        self.map_resolution = { 'x': 32, 'y': 32, 'z': 32 }

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
            vector3 = trans.transform.translation
            pos: Dict[str, float] = { 'x': vector3.x, 'y': vector3.y, 'z': vector3.z }
            return pos
        except Exception as e:
            self.get_logger().error(f'Error getting relative position: {e}')
            raise e

    def _callback(self, data: PolygonStamped):
        print("Data received.")
        start = datetime.now()

        inputdata = {
            'vertices': [ { 'x': p.x, 'y': p.y, 'z': p.z } for p in data.polygon.points ],
            'resolution': self.map_resolution,
            'me': self.getLocation('kevin', 'leo02/map'),
            'others': [
                # generically get all positions?
                # I.e. do no hard code individual robot's channels?
                self.getLocation('leo02/base_link', 'leo02/map'),
            ],
        }
        print("Retrieved positions & formulated input data ... ({})".format(datetime.now() - start))
        print(f'inputdata: {inputdata}')

        r = requests.post(f'{self.api_url}/{self.api_route}', json=inputdata)
        print("Made query ... ({})".format(datetime.now() - start))

        if r.status_code != 200:
            print("Error occurred:", r)
            print(r.text)
            return

        jdata = json.loads(r.text)
        print("Parsed JSON ({})".format(datetime.now() - start))

        print("{} cells were returned.".format(len(jdata['cells'])))
        print("Offset: {}".format(len(jdata['offset'])))
        print("Resolution: {}".format(len(jdata['resolution'])))
        first_few_cells = 10
        print(f"First {first_few_cells} cells:")
        for d in jdata['cells'][:first_few_cells]:
            print(d)
