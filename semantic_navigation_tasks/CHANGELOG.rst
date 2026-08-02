^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_navigation_tasks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (02-08-2026)
------------------
* Bound the number of sampling attempts in ``generateRandomGoals`` and validate the requested ``n`` and the region bounding box, so an unreachable region can no longer hang the service while holding its mutex.
* Fix an out-of-bounds read in ``getRandomRegionService`` on an empty region list, and an off-by-one in ``cell()``'s bounds check.
* Propagate the requested ``orientation``/``yaw`` through ``generateRandomGoals`` instead of always ignoring them and forcing ``INSIDE``.
* Use ``double`` precision (instead of truncating to ``int``) for the map bounds in ``processBoundingBox``.
* Read the regions file once instead of twice per ``on_configure``, and validate that each region has at least 3 points with 2 coordinates.
* Guard ``Region::isPointAtLeastDistanceFromBorders`` against a degenerate (0 or 1 point) polygon.
* Pass ``Region``/``vector<Region>``/``Pose`` by ``const&`` instead of by value in the sampling hot path.
* Unify the point-to-segment distance calculation (previously duplicated) into ``geometry_utils.hpp``.
* Add the ``global_frame`` parameter, decoupled from ``map_topic``, used as ``header.frame_id`` for published messages.
* Add ``success``/``message`` to the responses of all 7 services.
* Downgrade recoverable service failures from ``FATAL`` to ``WARN``/``ERROR``.

3.3.0 (29-06-2026)
------------------
* Add a connectivity graph between regions with automatic detection from the geometry and manual ``add``/``remove`` overrides from the regions file.
* Add the ``get_adjacent_regions``, ``are_regions_connected`` and ``get_region_route`` services to query the connectivity graph.
* Publish the connectivity edges as markers for visualization.
* Add the ``auto_connect``, ``connectivity_threshold`` and ``edges_topic`` parameters.

3.2.0 (12-02-2026)
------------------
* Add TF2 listener to transform request points to map frame.

3.1.1 (06-02-2025)
------------------
* First jazzy release.

3.1.0 (29-01-2025)
------------------
* Replace the goals array with a vector of PoseStamped messages. TODO: Replace to PoseStampedArray in Kilted.
* Add a new service to get a random region.

3.0.1 (31-07-2024)
------------------
* Update to use modern CMake idioms.

3.0.0 (03-06-2024)
------------------
* Update License to Apache 2.0.
* Update CMakelists.txt and package.xml with new compilation flags.
* Improve format and style.
* Update documentation.
* Converted to Lifecycle node.
* Converted to component.
* Added composable nodes in launch file.
* Rename ROIS to regions.

2.4.1 (16-05-2024)
------------------
* Fix errors and bump to version of slg_msgs 3.9.0.

2.4.0 (03-11-2023)
------------------
* Added service for SemanticRegions.

2.3.1 (30-10-2023)
------------------
* Added rois_filename to launch file.

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
