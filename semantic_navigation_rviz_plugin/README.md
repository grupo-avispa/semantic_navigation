semantic_navigation_rviz_plugin
==========================

Panel for Rviz to send the robot to a region of interest (ROI) and tool to save ROIs in a YAML file.

Usage of Semantic Navigation Panel
=====

Open Rviz and add the new panel. Then, write the room in the textbox and clic on "Send the robot to the room".

If you want to know where the robot is, clic on "Where is the robot?".

Usage of Semantic Annotation Tool
=====

* Open the plugin in Rviz by clicking "+" in the tool panel and select "Semantic Annotation".
* In the displays panel, make sure you have the topic "/rois_viz" of type PolygonArray.
* Write the name of the ROI in the text box.
* Clic with the left button on the map to add new points to a polygon.
* Clic with the right button to begin with a new polygon.
* Clic with the middle button to erase all polygons and start over.
