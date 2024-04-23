# semantic_navigation_msgs

## Overview
The `semantic_navigation_msgs` package provides a set of services that are utilized by the `semantic_navigation` package. These services are designed to facilitate navigation tasks in a semantic context, where regions are defined by names rather than purely spatial coordinates.

## Services (.srv)
* [GenerateRandomGoals](srv/GenerateRandomGoals.srv): This service generates random goals within named regions. This is particularly useful for tasks that require random exploration within specific regions.
* [GetRegionName](srv/GetRegionName.srv): Given a spatial position, this service returns the name of the region that enclose the provided position. This allows for semantic interpretation of spatial data.
* [ListAllRegions](srv/ListAllRegions.srv): This service returns a list of all named regions. This can be used to get an overview of all the regions defined in the semantic context.