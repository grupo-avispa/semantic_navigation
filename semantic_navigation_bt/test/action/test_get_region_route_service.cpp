// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/utils/test_service.hpp"
#include "semantic_navigation_bt/action/get_region_route_service.hpp"
#include "semantic_navigation_msgs/srv/get_region_route.hpp"

class GetRegionRouteService : public TestService<semantic_navigation_msgs::srv::GetRegionRoute>
{
public:
  GetRegionRouteService()
  : TestService("get_region_route")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<semantic_navigation_msgs::srv::GetRegionRoute::Request> request,
    const std::shared_ptr<semantic_navigation_msgs::srv::GetRegionRoute::Response> response)
  {
    (void)request_header;
    (void)request;
    response->route = {"region1", "region2", "region3"};
    response->success = true;
  }
};

class GetRegionRouteServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_region_route_test_fixture");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>(
      "server_timeout", std::chrono::milliseconds(20));
    config_->blackboard->set<std::chrono::milliseconds>(
      "bt_loop_duration", std::chrono::milliseconds(10));
    config_->blackboard->set<std::chrono::milliseconds>(
      "wait_for_service_timeout", std::chrono::milliseconds(1000));

    factory_->registerNodeType<semantic_navigation_bt::GetRegionRouteService>("GetRegionRoute");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<GetRegionRouteService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GetRegionRouteServiceTestFixture::node_ = nullptr;
std::shared_ptr<GetRegionRouteService> GetRegionRouteServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * GetRegionRouteServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetRegionRouteServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetRegionRouteServiceTestFixture::tree_ = nullptr;

TEST_F(GetRegionRouteServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GetRegionRoute service_name="get_region_route" start_region="region1" goal_region="region3" route="{route}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  auto route = tree_->rootBlackboard()->get<std::vector<std::string>>("route");
  EXPECT_EQ(route.size(), 3);
  EXPECT_EQ(route[0], "region1");
  EXPECT_EQ(route[1], "region2");
  EXPECT_EQ(route[2], "region3");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  GetRegionRouteServiceTestFixture::server_ = std::make_shared<GetRegionRouteService>();
  std::thread server_thread([]() {
      rclcpp::spin(GetRegionRouteServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
