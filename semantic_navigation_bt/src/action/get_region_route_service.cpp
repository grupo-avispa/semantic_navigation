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

#include <memory>
#include <string>
#include <vector>

#include "semantic_navigation_bt/action/get_region_route_service.hpp"

namespace semantic_navigation_bt
{

GetRegionRouteService::GetRegionRouteService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf)
: BtServiceNode<semantic_navigation_msgs::srv::GetRegionRoute>(service_node_name, conf)
{
}

void GetRegionRouteService::on_tick()
{
  getInput("start_region", request_->start_region);
  getInput("goal_region", request_->goal_region);
}

BT::NodeStatus GetRegionRouteService::on_completion(
  std::shared_ptr<semantic_navigation_msgs::srv::GetRegionRoute::Response> response)
{
  if (!response->success || response->route.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("route", response->route);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace semantic_navigation_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<semantic_navigation_bt::GetRegionRouteService>("GetRegionRoute");
}
