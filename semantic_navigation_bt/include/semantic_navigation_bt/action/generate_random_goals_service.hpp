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

#ifndef SEMANTIC_NAVIGATION_BT__ACTION__GENERATE_RANDOM_GOALS_SERVICE_HPP_
#define SEMANTIC_NAVIGATION_BT__ACTION__GENERATE_RANDOM_GOALS_SERVICE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"

namespace semantic_navigation_bt
{

using nav2_behavior_tree::BtServiceNode;

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps scitos_msgs::srv::GenerateRandomGoals
 */
class GenerateRandomGoalsService
  : public BtServiceNode<semantic_navigation_msgs::srv::GenerateRandomGoals>
{
public:
  /**
   * @brief A constructor for semantic_navigation_bt::GenerateRandomGoals Service
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  GenerateRandomGoalsService(
    const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Override the on_completion method to set the output port with the goals generated after
   * the completion of the service.
   * @param response The response from the service
   * @return BT::NodeStatus Returns SUCCESS if the goals are generated correctly
   */
  BT::NodeStatus on_completion(
    std::shared_ptr<semantic_navigation_msgs::srv::GenerateRandomGoals::Response> response) override; //NOLINT

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<int>("number_of_goals", "Number of goals to generate"),
        BT::InputPort<std::string>("region_name", "Region name"),
        BT::InputPort<float>("yaw", "Yaw of the goals (Optional)"),
        BT::InputPort<std::string>("orientation", "Random orientation of the goals"),
        BT::InputPort<float>("border", 0.0, "Distance from the edge of the region"),
        BT::OutputPort<geometry_msgs::msg::PoseArray>("goals", "Generated goals"),
      });
  }
};

}  // namespace semantic_navigation_bt

#endif  // SEMANTIC_NAVIGATION_BT__ACTION__GENERATE_RANDOM_GOALS_SERVICE_HPP_
