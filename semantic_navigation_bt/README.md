# semantic_navigation_bt

## Overview

This package provides several behavior tree plugins for the semantic navigation stack. These plugins are designed to facilitate the control and management of the robot's actions and responses. The behavior tree nodes are implemented using the [BehaviorTree.CPP] library, which provides the core functionality for behavior tree processing.

The plugins included in this package are:

* **GenerateRandomGoals**: This node generates random goals within a specified region. The region is defined by the name of the polygon, which is read from a configuration file. The node publishes the generated goals to a topic, which can be used by the robot's navigation system to navigate to the goal.
* **GetRegionName**: This node retrieves the name of the region in which a given position is located. The position is provided as input to the node, and the node returns the name of the region in which the position is located.
* **ListAllRegions**: This node lists all the regions defined in the configuration file. The node reads the configuration file and returns a list of all the regions defined in the file.

[BehaviorTree.CPP]: https://github.com/BehaviorTree/BehaviorTree.CPP