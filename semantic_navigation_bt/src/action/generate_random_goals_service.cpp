// Copyright (c) 2024 Alberto J. Tudela Roldán
// Copyright (c) 2024 Grupo Avispa, DTE, Universidad de Málaga
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

#include <string>
#include <memory>

#include "semantic_navigation_bt/action/generate_random_goals_service.hpp"

namespace semantic_navigation_bt
{

GenerateRandomGoalsService::GenerateRandomGoalsService(
  const std::string & service_node_name, const BT::NodeConfiguration & conf)
: BtServiceNode<GenerateRandomGoals>(service_node_name, conf)
{
}

void GenerateRandomGoalsService::on_tick()
{
  getInput("number_of_goals", request_->n);
  getInput("region_name", request_->region_name);
  getInput("yaw", request_->yaw);
  getInput("orientation", request_->orientation);
  getInput("border", request_->border);
}

BT::NodeStatus GenerateRandomGoalsService::on_completion(
  std::shared_ptr<GenerateRandomGoals::Response> response)
{
  BT::NodeStatus status = BT::NodeStatus::FAILURE;
  if (response->goals.size() > 0) {
    setOutput("goals", response->goals);
    status = BT::NodeStatus::SUCCESS;
  }
  return status;
}

}  // namespace semantic_navigation_bt

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<semantic_navigation_bt::GenerateRandomGoalsService>(
    "GenerateRandomGoals");
}
