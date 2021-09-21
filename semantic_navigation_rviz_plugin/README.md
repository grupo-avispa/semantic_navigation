# semantic_navigation_rviz_plugin

![ROS](https://img.shields.io/badge/ros-melodic-blue?style=for-the-badge&logo=ros&logoColor=white)

## Overview

Panel for [RViz] to send the robot to a region of interest (ROI) and tool to save ROIs in a YAML file.

## Usage

For the Semantic Navigation Panel:

* Open Rviz and add the new panel. Then, write the room in the textbox and clic on "Send the robot to the room".
* By default it will send one (1) goal with orientation inside the ROI and at 0.1m from the border.

If you want to know where the robot is, clic on "Where is the robot?".

For the Semantic Annotation Tool:

* Open the plugin in Rviz by clicking "+" in the tool panel and select "Semantic Annotation".
* In the Displays panel, add a display of jsk_rviz_plugins/PolygonArray type and make sure the name of the topic is "/rois_viz".
* In the Displays panel, add a display of rviz/MarkerArray type and make sure the name of the topic is "/rois_names_viz".
* In the Tool properties panel, you can change the inflation radius, write the ROIs names splitted by a comma (,) and the YAML configuration filename. It will store in the config folder of semantic_goals_generator package.
* Clic with the left button on the map to add new points to a polygon.
* Clic with the right button to start a new polygon.
* Clic with the middle button to erase all polygons, save them and start over.


[Rviz]: http://wiki.ros.org/rviz
