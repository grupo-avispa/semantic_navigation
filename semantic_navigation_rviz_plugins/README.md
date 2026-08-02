# semantic_navigation_rviz_plugins

## Overview

Panel for [RViz2] to send the robot to a region and tool to save regions in a YAML file.

## Usage

For the Semantic Navigation Panel:

* Open Rviz2 and add the new panel. Then, write the room in the textbox and click on "Send the robot to the room".
* By default it will send one (1) goal with orientation inside the Region and at 0.1m from the border.

If you want to know where the robot is, click on "Where is the robot?".

For the Semantic Annotation Tool:

* Open the plugin in Rviz2 by clicking "+" in the tool panel and select "Semantic Annotation".
* In the Displays panel, add a display of polygon_rviz_plugins/Polygons type and make sure the name of the topic is "/polygons".
* In the Displays panel, add a display of rviz/MarkerArray type and make sure the name of the topic is "/names".
* In the Tool properties panel, you can change the inflation radius, write the regions names splitted by a comma (,), the YAML configuration filename and the save path (a writable directory; defaults to your home folder). The file is saved as `<save path>/<filename>.yaml`.
* Click with the left button on the map to add new points to a polygon.
* Click with the right button to start a new polygon.
* Click with the middle button to erase all polygons, save them and start over.

[Rviz2]: https://github.com/ros2/rviz