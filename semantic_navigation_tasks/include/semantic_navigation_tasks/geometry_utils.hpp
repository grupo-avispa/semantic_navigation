// Copyright (c) 2026 Alberto J. Tudela Roldán
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef SEMANTIC_NAVIGATION_TASKS__GEOMETRY_UTILS_HPP_
#define SEMANTIC_NAVIGATION_TASKS__GEOMETRY_UTILS_HPP_

#include <algorithm>
#include <cmath>

namespace semantic_navigation::geometry_utils
{

/**
 * @brief Compute the distance between a point and a segment.
 *
 * The segment is allowed to be degenerate (its two endpoints coincide), in which case the
 * distance to that single point is returned instead of dividing by zero.
 *
 * @param px X coordinate of the point.
 * @param py Y coordinate of the point.
 * @param x0 X coordinate of the first end of the segment.
 * @param y0 Y coordinate of the first end of the segment.
 * @param x1 X coordinate of the second end of the segment.
 * @param y1 Y coordinate of the second end of the segment.
 * @return Distance between the point and the segment.
 */
inline double pointToSegmentDistance(
  double px, double py, double x0, double y0, double x1, double y1)
{
  double dx = x1 - x0;
  double dy = y1 - y0;
  double len_sq = dx * dx + dy * dy;

  // Degenerate segment (a single point)
  if (len_sq < 1e-12) {
    return std::hypot(px - x0, py - y0);
  }

  double param = ((px - x0) * dx + (py - y0) * dy) / len_sq;
  param = std::clamp(param, 0.0, 1.0);

  double xx = x0 + param * dx;
  double yy = y0 + param * dy;
  return std::hypot(xx - px, yy - py);
}

}  // namespace semantic_navigation::geometry_utils

#endif  // SEMANTIC_NAVIGATION_TASKS__GEOMETRY_UTILS_HPP_
