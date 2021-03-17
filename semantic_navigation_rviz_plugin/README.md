semantic_navigation_rviz_plugin
==========================

Panel for Rviz to send the robot to a region of interest (ROI) and tool to save ROIs in a YAML file.

Usage of Semantic Navigation Panel
=====

Open Rviz and add the new panel. Then, write the room in the textbox and clic on "Send the robot to the room".
By default it will send one (1) goal with orientation inside the ROI and at 0.1m from the border.

If you want to know where the robot is, clic on "Where is the robot?".

Usage of Semantic Annotation Tool
=====

* Open the plugin in Rviz by clicking "+" in the tool panel and select "Semantic Annotation".
* In the Displays panel, make sure you have the topic "/rois_viz" of type PolygonArray and "/rois_names_viz" of type MarkerArray.
* In the Tool properties panel, change the inflation radius and write the ROIs names splitted by a comma (,).
* Clic with the left button on the map to add new points to a polygon.
* Clic with the right button to start a new polygon.
* Clic with the middle button to erase all polygons, save them and start over.
