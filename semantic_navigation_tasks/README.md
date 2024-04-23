# semantic_navigation_tasks

## Overview

ROS Service to generate 2D navigation goals with orientation in a specifed region of interest (ROI). These ROIs are described by a polygon 
defined by its edges in the map frame and a name. The service takes the number of navigation goals (*n*) and a ROI name (*roi_name*) and 
returns a list of goal poses. 

The ROIs are stored in a YAML configuration file. Look for the examples to get more information about it.

There are optional parameters like:
- Direction of the goal. The goal can be orientated `outside` the ROI, `inside` the ROI, the value `stored` in the configuration file, `requested` (see below) or `random` by default.
- Yaw. Alternatively to the direction of the goal, the yaw of the goals can be set to a value between -PI and PI. This value can be set into the configuration file or sent using the service.
- Distance from the border of the ROI. The goal can be at a distance (in meters) from the border of the ROI. Default is 0.0.

In addition to the random navigation goals service, it's also included:
- A service to request the ROI name of a known position.
- A latched publisher of the ROI name where the robot is.

## Usage

For the goals generator service, launch the node as follows:

	ros2 launch semantic_navigation_tasks semantic_navigation_tasks.launch

You can send a service to request goals as follows:

	ros2 service call /semantic_goals '{n: 1, roi_name: "roi_0", direction: "inside", border: 0.1}'

whereby the first argument is the number of goal locations to be generated (here 1), the second argument is the name of a ROI specified that match the list in the configuration file (here roi_0), the orientation of the goals (here inside) and the distance from the border of the ROI (here 0.1). 
The result of the pose generation is additionally published on the topic `/semantic_goals` in order to visualize the result in [RViz].

If the service is called with an empty ROI or the ROI is not in the configuration file, the full map is considered as ROI by default. 

	ros2 service call /semantic_goals '{n: 100, roi_name: {}, direction: "random", border: 0.0}'


If a specified ROI includes a point that is outside the map, its *conflicting* coordinates are automatically adjusted to the map's bounding box.

For the position service, to know the name of the ROI where the robot is, send the service request as follows:

	ros2 service call /semantic_position '{position.x: 0.0, position.y: 0.0, position.z: 0.0}'

## Nodes

### semantic_navigation_tasks

ROS2 Service to generate 2D navigation goals as described above.


#### Subscribed Topics

* **`map`** ([nav_msgs/OccupancyGrid])

	The map where the robot moves.

#### Published Topics

* **`semantic_goals`** ([geometry_msgs/PoseArray])

	Topic where the random navigation goals are published.

* **`polygons`** ([polygon_msgs/Polygon2DCollection])

	Topic array with filled polygons of the Regions of Interest (ROIs).

* **`names`** ([visualization_msgs/MarkerArray])

	Topic array with the names of the Regions of Interest (ROIs).

#### Services

* **`semantic_goals`** ([semantic_navigation_msgs/SemanticGoals])

	Topic where the random navigation goals are published.

* **`semantic_position`** ([semantic_navigation_msgs/SemanticPosition])

	Topic where the semantic position of the robot is published.

#### Parameters

* **`goals_topic`** (string, default: "semantic_goals")

	Topic where the random navigation goals are published.

* **`polygons_topic`** (string, default: "polygons")

	Topic array with filled polygons of the Regions of Interest (ROIs).

* **`names_topic`** (string, default: "names")

	Topic array with the names of the Regions of Interest (ROIs).

* **`map_topic`** (string, default: "map")

	Topic of the map where the robot moves.

* **`is_costmap`** (bool, default: false)

	If the map argument is a costmap, you should also set the flag `is_costmap` to `true`. Then the inflation radius in the service call is ignored (a costmap is already inflated)

* **`full_map`** (bool, default: false)

	Option to choose the full map if a requested ROI is not found in the configuration file or reject the goal request.

* **`inflation_radius`** (float, default: 0.5)

	The inflation radius of the robot's footprint.

* **`rois`** (string, default: "rois.yaml")

	The filepath of the configuration file including the names of regions of interests (ROIs) defined by its edges and the inflation radius of the robot's footprint as above.


[nav_msgs/OccupancyGrid]: https://docs.ros2.org/humble/api/nav_msgs/msg/OccupancyGrid.html
[geometry_msgs/PoseArray]: https://docs.ros2.org/humble/api/geometry_msgs/msg/PoseArray.html
[polygon_msgs/Polygon2DCollection]: https://github.com/MetroRobots/polygon_ros/blob/main/polygon_msgs/msg/Polygon2DCollection.msg
[visualization_msgs/MarkerArray]: https://docs.ros2.org/humble/api/visualization_msgs/msg/MarkerArray.html
[semantic_navigation_msgs/SemanticGoals]: ../semantic_navigation_msgs/srv/SemanticGoals.srv
[semantic_navigation_msgs/SemanticPosition]: ../semantic_navigation_msgs/srv/SemanticPosition.srv