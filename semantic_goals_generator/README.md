# semantic_goals_generator

![ROS](https://img.shields.io/badge/ros-melodic-blue?style=for-the-badge&logo=ros&logoColor=white)

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

**Keywords:** ROS, navigation, semantic, social

### License

**Author: Alberto Tudela<br />**

The semantic_goals_generator package has been tested under [ROS] Melodic on [Ubuntu] 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [simple_laser_geometry](https://github.com/ajtudela/simple_laser_geometry) (Library and messages to interact with laser related geometry),
- [jsk_recognition_msgs](https://jsk-visualization.readthedocs.io/en/latest/index.html) (jsk_visualization)

	sudo rosdep install --from-paths src

#### Building

To build from source, clone the latest version from the main repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://gitlab.com/ajtudela/semantic_navigation.git
	cd ../
	rosdep install --from-paths . --ignore-src
	catkin_make

## Usage

For the goals generator service, launch the node as follows:

	roslaunch semantic_goals_generator semantic_goals_generator.launch

You can send a service to request goals as follows:

	rosservice call /semantic_goals '{n: 1, roi_name: "roi_0", direction: "inside", border: 0.1}'

whereby the first argument is the number of goal locations to be generated (here 1), the second argument is the name of a ROI specified that match the list in the configuration file (here roi_0), the orientation of the goals (here inside) and the distance from the border of the ROI (here 0.1). 
The result of the pose generation is additionally published on the topic `/semantic_goals` in order to visualize the result in [RViz].

If the service is called with an empty ROI or the ROI is not in the configuration file, the full map is considered as ROI by default. 

	rosservice call /semantic_goals '{n: 100, roi_name: {}, direction: "random", border: 0.0}'


If a specified ROI includes a point that is outside the map, its *conflicting* coordinates are automatically adjusted to the map's bounding box.

For the position service, to know the name of the ROI where the robot is, send the service request as follows:

	rosservice call /semantic_position '{position.x: 0.0, position.y: 0.0, position.z: 0.0}'

## Nodes

### semantic_goals_generator

ROS Service to generate 2D navigation goals as described above.


#### Subscribed Topics

* **`map`** ([nav_msgs/OccupancyGrid])

	The map where the robot moves.

* **`robot_pose`** ([geometry_msgs/PoseStamped])

	The pose of the robot.

#### Published Topics

* **`semantic_goals`** ([geometry_msgs/PoseArray])

	Topic where the random navigation goals are published.

* **`semantic_position`** ([std_msgs/String])

	Topic where the semantic location of the robot is published.

* **`rois_viz`** ([jsk_recognition_msgs/PolygonArray])

	Topic array with filled polygons of the Regions of Interest (ROIs).

* **`rois_names_viz`** ([visualization_msgs/MarkerArray])

	Topic array with the names of the Regions of Interest (ROIs).

#### Parameters

* **`is_costmap`** (bool, default: false)

	If the map argument is a costmap, you should also set the flag `is_costmap` to `true`. Then the inflation radius in the service call is ignored (a costmap is already inflated)

* **`full_map`** (bool, default: false)

	Option to choose the full map if a requested ROI is not found in the configuration file or reject the goal request.

* **`inflation_radius`** (float, default: 0.5)

	The inflation radius of the robot's footprint.

* **`rois`** (string, default: "rois.yaml")

	The filepath of the configuration file including the names of regions of interests (ROIs) defined by its edges and the inflation radius of the robot's footprint as above.



[Ubuntu]: https://ubuntu.com/
[ROS]: http://www.ros.org
[Rviz]: http://wiki.ros.org/rviz
[std_msgs/String]: http://docs.ros.org/api/std_msgs/html/msg/String.html
[nav_msgs/OccupancyGrid]: http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
[geometry_msgs/PoseArray]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html
[geometry_msgs/PoseStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
[visualization_msgs/MarkerArray]: http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
[jsk_recognition_msgs/PolygonArray]: http://docs.ros.org/api/jsk_recognition_msgs/html/msg/PolygonArray.html
