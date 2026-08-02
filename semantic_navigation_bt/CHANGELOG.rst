^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package semantic_navigation_bt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.4.0 (02-08-2026)
------------------
* Update the 7 action nodes to check the new ``response->success`` field, instead of inferring success only from whether the result happens to be non-empty.
* Fix ``GetRegionNameService`` treating the ``UNKNOWN`` sentinel region name as a successful tick.

3.3.0 (29-06-2026)
------------------
* Added "get_adjacent_regions" action BT node.
* Added "are_regions_connected" action BT node.
* Added "get_region_route" action BT node.

3.2.0 (12-01-2026)
------------------
* Change GetRegionName.srv to use PointStamped instead of Point.

3.1.1 (06-02-2025)
------------------
* First jazzy release.
* Move to BehaviorTree 4.6.

3.1.0 (29-01-2025)
------------------
* Replace the goals array with a vector of PoseStamped messages. TODO: Replace to PoseStampedArray in Kilted.
* Add a new service to get a random region.

3.0.1 (31-07-2024)
------------------
* Update to use modern CMake idioms.

3.0.0 (03-06-2024)
------------------
* Create README.md.
* Create CHANGELOG.rst.
* Create package.xml.
* Added "generate_random_goals" action BT node.
* Added "get_region_name" action BT node.
* Added "list_all_regions" action BT node.
* Contributors: Alberto Tudela.
