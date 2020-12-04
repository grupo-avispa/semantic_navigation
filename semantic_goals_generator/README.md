semantic_goals_generator
========================

ROS Service to generate 2D navigation goals with orientation in a specifed
region of interest (ROI). The service takes the number of navigation goals
(*n*) and a ROI described by a polygon as arguments and returns a list of goal poses.

Also has a service to request the ROI name of a position. 

Usage
=====

Launch the node as follows:
```
roslaunch semantic_goals_generator semantic_goals_generator.launch
```

You can send a service to request goals as follows:
```
rosservice call /semantic_goals '{n: 1, roi_name: "livingroom"}'
```

whereby the first argument is the number of goal locations to be generated
(here 1) and the second argument is the name of a ROI specified as a list of
edges (at least three) in the configuration file.  The result of the pose generation is additionally
published on the topic `/semantic_goals` in order to visualize the result in RVIZ.

If the service is called with an empty ROI, the full map is considered as ROI
by default. 

```
rosservice call /semantic_goals '{n: 100, roi_name: {}}'
```

If a specified ROI includes a point that is outside the map, its *conflicting*
coordinates are automatically adjusted to the map's bounding box.

To know the name of the ROI where the robot is, send the service request as follows:
```
rosservice call /semantic_position '{position.x: 0.0, position.y: 0.0, position.z: 0.0}'
```

Parameters
----------
 * ```~is_costmap```
  [bool, default:false]
  If the map argument is a costmap, you should also set the flag `is_costmap` to `true`. 
  Then the inflation radius in the service call is ignored (a costmap is already inflated)
 * ```~inflation_radius```
  [float, default:0.5]
  The inflation radius of the robot's footprint.
 * ```rois.yaml```
  A configuration file including the names of regions of interests (ROIs) and the points defining them.

Subscriptions
----------
 * ```map_frame```
  [nav_msgs/OccupancyGrid, default:map]
  The map frame where the robot moves.
 
Publications
----------
 * ```semantic_goals```
  [geometry_msgs/PoseArray]
  Random navigation goals generated. 
 * ```rois_viz```
  [jsk_recognition_msgs/PolygonArray]
  Filled polygons array of Regions of Interest.
 * ```rois_names_viz```
  [visualization_msgs/MarkerArrayy]
  Names array of Regions of Interest.
