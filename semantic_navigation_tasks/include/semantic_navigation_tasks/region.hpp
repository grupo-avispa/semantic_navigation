// Copyright (c) 2020 Alberto J. Tudela Roldán
// Copyright (c) 2020 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SEMANTIC_NAVIGATION_TASKS__REGION_HPP_
#define SEMANTIC_NAVIGATION_TASKS__REGION_HPP_

#include <math.h>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "polygon_utils/polygon_utils.hpp"
#include "polygon_msgs/msg/polygon2_d.hpp"

namespace semantic_navigation
{

/**
 * @struct semantic_navigation::Region
 * @brief Region of interest.
 */
struct Region
{
  std::string name;  // Name of the region of interest
  polygon_msgs::msg::Polygon2D polygon;  // Polygon defining the region of interest

  inline bool empty() {return polygon.points.empty();}
  inline void clear() {return polygon.points.clear();}
  inline int size() {return polygon.points.size();}

  inline geometry_msgs::msg::Point centroid() const
  {
    geometry_msgs::msg::Point centroid;
    centroid.x = 0;
    centroid.y = 0;
    for (auto & point : polygon.points) {
      centroid.x += point.x;
      centroid.y += point.y;
    }
    centroid.x /= polygon.points.size();
    centroid.y /= polygon.points.size();
    return centroid;
  }

  /**
   * @brief Checks if point is inside polygon.
   * @param point Given point to check
   * @return True if given point is inside polygon, otherwise false
   */
  inline bool isPointInside(const double x, const double y) const
  {
    return polygon_utils::isInside(polygon, x, y);
  }

  /**
   * @brief Check if the point is at least a certain distance from all borders.
   * @param x X coordinate of the point
   * @param y Y coordinate of the point
   * @param distance Minimum distance from the borders
   * @return bool if given point is inside polygon
   */
  bool isPointAtLeastDistanceFromBorders(float x, float y, float distance)
  {
    // A degenerate polygon (0 or 1 points) has no borders to measure against. Returning true
    // avoids the unsigned underflow of `size() - 1` below and the resulting out-of-bounds
    // access to front()/back() on an empty vector.
    if (polygon.points.size() < 2) {
      return true;
    }

    for (unsigned int i = 0; i < polygon.points.size() - 1; i++) {
      if (distanceToLine(
          x, y,
          polygon.points[i].x, polygon.points[i].y,
          polygon.points[i + 1].x, polygon.points[i + 1].y) < distance)
      {
        return false;
      }
    }
    // Check distance from the last point to the first point
    if (distanceToLine(
        x, y,
        polygon.points.back().x, polygon.points.back().y,
        polygon.points.front().x, polygon.points.front().y) < distance)
    {
      return false;
    }

    return true;
  }

  double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
  {
    double A = pX - x0;
    double B = pY - y0;
    double C = x1 - x0;
    double D = y1 - y0;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
      xx = x0;
      yy = y0;
    } else if (param > 1) {
      xx = x1;
      yy = y1;
    } else {
      xx = x0 + param * C;
      yy = y0 + param * D;
    }

    return std::hypot(xx - pX, yy - pY);
  }
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__REGION_HPP_
