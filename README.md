# About

A ROS2 Python client meant to be run on a rover, and making use of our [`partition-api`][pa]. It effectively embeds the [python client][pc]'s POST request method into a ROS2 node.

[pa]: https://github.com/ISM-Thesis-MultiRobot-Partitioning/partition-api
[pc]: https://github.com/ISM-Thesis-MultiRobot-Partitioning/python-client

This one has a few environment variables which control specifics of the setup.
| Variable | Description | Example |
| --- | --- | --- |
| API_URL | Where to reach the [partition-api][pa]? | `http://localhost:8080` |
| API_ROUTE | Which partitioning handler to use? Check the HTTP routes in the [main.rs][pa] file | `PolygonToCellMapContours` |
| MY_ODOM_TOPIC | The topic on which my own robot's pose can be retrieved | `/leo02/pose` |
| OTHER_ODOM_TOPICS | List of space separated topics for all other robot poses | `"/leo03/pose /leo07/pose /leo09/pose"` |
| INPUT_CHANNEL | Topic on which to receive the polygon message | `/exploration_zone` |
| OUTPUT_CHANNEL | Topic on which to publish the region assigned by the [partition-api][pa] | `/assigned_border` |

