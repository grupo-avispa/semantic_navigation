^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_navigation_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.0 (12-01-2026)
------------------
* Change GetRegionName.srv to use PointStamped instead of Point.

3.1.1 (06-02-2025)
------------------
* First jazzy release.

3.1.0 (29-01-2025)
------------------
* Replace the goals array with a vector of PoseStamped messages. TODO: Replace to PoseStampedArray in Kilted.
* Add a new service to get a random region.

3.0.0 (03-06-2024)
------------------
* Update License to Apache 2.0.
* Update CMakelists.txt and package.xml with new compilation flags.
* Improve format and style.
* Update documentation.
* Rename services.

1.1.0 (03-11-2023)
------------------
* Added SemanticRegions.srv action file.

1.0.0 (25-07-2023)
------------------
* Move from semantic_goals_generator (2.2.0)
* First ROS2 (Humble) version.