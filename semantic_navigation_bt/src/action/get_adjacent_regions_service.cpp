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

#include "semantic_navigation_bt/action/get_adjacent_regions_service.hpp"

namespace semantic_navigation_bt
{

GetAdjacentRegionsService::GetAdjacentRegionsService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf)
: BtServiceNode<semantic_navigation_msgs::srv::GetAdjacentRegions>(service_node_name, conf)
{
}

void GetAdjacentRegionsService::on_tick()
{
  getInput("region_name", request_->region_name);
}

BT::NodeStatus GetAdjacentRegionsService::on_completion(
  std::shared_ptr<semantic_navigation_msgs::srv::GetAdjacentRegions::Response> response)
{
  setOutput("adjacent_regions", response->adjacent_regions);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace semantic_navigation_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<semantic_navigation_bt::GetAdjacentRegionsService>("GetAdjacentRegions");
}
