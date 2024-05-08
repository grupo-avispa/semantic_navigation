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

#ifndef SEMANTIC_NAVIGATION_BT__ACTION__LIST_ALL_REGIONS_SERVICE_HPP_
#define SEMANTIC_NAVIGATION_BT__ACTION__LIST_ALL_REGIONS_SERVICE_HPP_

#include <memory>
#include <string>

#include "nav2_behavior_tree/bt_service_node.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

namespace semantic_navigation_bt
{

using nav2_behavior_tree::BtServiceNode;

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps semantic_navigation_msgs::srv::ListAllRegions
 */
class ListAllRegionsService : public BtServiceNode<semantic_navigation_msgs::srv::ListAllRegions>
{
public:
  /**
   * @brief A constructor for semantic_navigation_bt::ListAllRegionsService
   * @param service_node_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  ListAllRegionsService(const std::string & service_node_name, const BT::NodeConfiguration & conf);

  /**
   * @brief The main override required by a BT service
   * @return BT::NodeStatus Status of tick execution
   */
  void on_tick() override;

  /**
   * @brief Override the on_completion method to set the output port with the region names after the
   * completion of the service.
   * @param response The response from the service
   * @return BT::NodeStatus Returns SUCCESS if the region names are retrieved correctly
   */
  BT::NodeStatus on_completion(
    std::shared_ptr<semantic_navigation_msgs::srv::ListAllRegions::Response> response) override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<std::string>("region_names", "Region names")
      });
  }
};

}  // namespace semantic_navigation_bt

#endif  // SEMANTIC_NAVIGATION_BT__ACTION__LIST_ALL_REGIONS_SERVICE_HPP_
