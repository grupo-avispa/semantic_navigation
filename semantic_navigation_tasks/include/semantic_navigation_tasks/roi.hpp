// Copyright (c) 2020 Alberto J. Tudela Roldán
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

#ifndef SEMANTIC_NAVIGATION_TASKS__ROI_HPP_
#define SEMANTIC_NAVIGATION_TASKS__ROI_HPP_

#include <string>

#include "slg_msgs/polygon.hpp"

namespace semantic_navigation
{

struct ROI
{
  slg::Polygon polygon;
  float yaw;

  inline bool empty() {return polygon.empty();}
  inline void clear() {return polygon.clear();}
  inline std::string get_name() {return polygon.get_name();}
  inline void set_name(std::string name) {polygon.set_name(name);}

  /* Check if a point is inside the region of interest (ROI) */
  bool in_roi(float x, float y)
  {
    if (polygon.size() == 0) {return true;}
    return polygon.contains(slg::Point2D(x, y));
  }

  /* Check if the point is at distance from all borders */
  bool distance_from_borders(float x, float y, float border)
  {
    for (auto & edge : polygon.get_edges()) {
      if (edge.distance(slg::Point2D(x, y)) < border) {return false;}
    }
    return true;
  }
};

}  // namespace semantic_navigation

#endif  // SEMANTIC_NAVIGATION_TASKS__ROI_HPP_
