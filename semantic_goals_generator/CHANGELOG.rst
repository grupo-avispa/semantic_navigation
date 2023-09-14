^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_goals_generator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (14-09-2023)
------------------
* First ROS2 (Humble) version.

2.2.0 (18-07-2022)
------------------
* Create subscriber to "robot_pose".
* Create publisher of "semantic_position".

2.1.0 (17-03-2022)
------------------
* Added option to choose between fullmap or reject.
* Fixed bounding box limits.

2.0.0 (18-02-2022)
------------------
* Change orientation subtopic to direction.
* Added yaw in srv message and in configuration file.

1.0.1 (15-12-2021)
------------------
* Update simple_laser_geometry to 3.0.0.

1.0.0 (20-09-2021)
------------------
* Create CHANGELOG.rst.

0.1.5 (05-07-2021)
------------------
* Remove upperbound limits.
* Change points for edges in polygon.
* Added border and orientation in service request.
* Change laser_utils for simple_laser_geometry library.

0.1.0 (25-11-2020)
------------------
* Added SemanticPosition.srv service.
* Fix nogo maps.

0.0.1 (02-07-2020)
------------------
* Create LICENSE.

0.0.0 (23-03-2020)
------------------
* Initial release.
* Create README.md.
* Added SemanticGoals.srv service.
* Added semantic_goals_generator class (.h and .cpp files).
* Added launch file
* Contributors: Alberto Tudela
