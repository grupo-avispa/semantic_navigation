^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_navigation_rviz_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (02-08-2026)
------------------
* Fix ``SemanticAnnotationTool::activate()`` recreating ``ros_node_`` and the publishers on every activation instead of only once in ``onInitialize()``.
* Save regions to a configurable, writable "Save path" using a ``YAML::Emitter`` (truncating the file), instead of appending hand-written YAML into another package's installed ``share/`` folder.

3.1.2 (08-01-2026)
------------------
* Fix ament_cpp API.
* Upgrade to Qt6.

3.1.1 (06-02-2025)
------------------
* First jazzy release.

3.1.0 (29-01-2025)
------------------
* Replace the goals array with a vector of PoseStamped messages. TODO: Replace to PoseStampedArray in Kilted.

3.0.1 (31-07-2024)
------------------
* Update to use modern CMake idioms.

3.0.0 (03-06-2024)
------------------
* Update License to Apache 2.0.
* Update CMakelists.txt and package.xml with new compilation flags.
* Improve format and style.
* Update documentation.
* Rename ROIS to regions.

1.0.4 (16-05-2024)
------------------
* Fix errors and bump to version of slg_msgs 3.9.0.

1.0.3 (18-05-2022)
------------------
* Clean list of ROIs names for spaces.

1.0.2 (18-02-2022)
------------------
* Change direction for constant message.

1.0.1 (15-12-2021)
------------------
* Update simple_laser_geometry to 3.0.0.

1.0.0 (20-09-2021)
------------------
* Create CHANGELOG.rst.

0.1.5 (05-07-2021)
------------------
* Added border and orientation in service request.
* Change laser_utils for simple_laser_geometry library.

0.1.0 (24-11-2020)
------------------
* Initial release.
* Create README.md.
* Added semanticNavigationPanel class (.h and .cpp files).
* Added semanticAnnotationTool class (.h and .cpp files).
* Contributors: Alberto Tudela
