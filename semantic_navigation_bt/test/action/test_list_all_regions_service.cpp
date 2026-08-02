// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/test/utils/test_service.hpp"
#include "semantic_navigation_bt/action/list_all_regions_service.hpp"
#include "semantic_navigation_msgs/srv/list_all_regions.hpp"

class ListAllRegionsService : public TestService<semantic_navigation_msgs::srv::ListAllRegions>
{
public:
  ListAllRegionsService()
  : TestService("list_all_regions")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<semantic_navigation_msgs::srv::ListAllRegions::Request> request,
    const std::shared_ptr<semantic_navigation_msgs::srv::ListAllRegions::Response> response)
  {
    (void)request_header;
    (void)request;
    response->region_names = {"region1", "region2", "region3"};
    response->success = true;
  }
};

class ListAllRegionsServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<nav2::LifecycleNode>("list_all_regions_test_fixture");
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

    factory_->registerNodeType<semantic_navigation_bt::ListAllRegionsService>("ListAllRegions");
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

  static std::shared_ptr<ListAllRegionsService> server_;

protected:
  static nav2::LifecycleNode::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

nav2::LifecycleNode::SharedPtr ListAllRegionsServiceTestFixture::node_ = nullptr;
std::shared_ptr<ListAllRegionsService> ListAllRegionsServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * ListAllRegionsServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> ListAllRegionsServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> ListAllRegionsServiceTestFixture::tree_ = nullptr;

TEST_F(ListAllRegionsServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <ListAllRegions service_name="list_all_regions" region_names="{region_names}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  auto region_names = tree_->rootBlackboard()->get<std::vector<std::string>>("region_names");
  EXPECT_EQ(region_names.size(), 3);
  EXPECT_EQ(region_names[0], "region1");
  EXPECT_EQ(region_names[1], "region2");
  EXPECT_EQ(region_names[2], "region3");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  ListAllRegionsServiceTestFixture::server_ = std::make_shared<ListAllRegionsService>();
  std::thread server_thread([]() {
      rclcpp::spin(ListAllRegionsServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
