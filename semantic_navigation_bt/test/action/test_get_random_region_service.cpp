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

#include "behaviortree_cpp_v3/bt_factory.h"

#include "utils/test_service.hpp"
#include "semantic_navigation_bt/action/get_random_region_service.hpp"
#include "semantic_navigation_msgs/srv/get_random_region.hpp"

class GetRandomRegionService : public TestService<semantic_navigation_msgs::srv::GetRandomRegion>
{
public:
  GetRandomRegionService()
  : TestService("get_random_region")
  {}

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<semantic_navigation_msgs::srv::GetRandomRegion::Request> request,
    const std::shared_ptr<semantic_navigation_msgs::srv::GetRandomRegion::Response> response)
  {
    (void)request_header;
    (void)request;
    response->region_name = "region1";
  }
};

class GetRandomRegionServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("get_random_region_test_fixture");
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

    factory_->registerNodeType<semantic_navigation_bt::GetRandomRegionService>("GetRandomRegion");
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

  static std::shared_ptr<GetRandomRegionService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GetRandomRegionServiceTestFixture::node_ = nullptr;
std::shared_ptr<GetRandomRegionService> GetRandomRegionServiceTestFixture::server_ = nullptr;
BT::NodeConfiguration * GetRandomRegionServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GetRandomRegionServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GetRandomRegionServiceTestFixture::tree_ = nullptr;

TEST_F(GetRandomRegionServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root>
        <BehaviorTree ID="MainTree">
            <GetRandomRegion service_name="get_random_region" region_name="{region_name}" />
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(tree_->rootBlackboard()->get<std::string>("region_name"), "region1");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  GetRandomRegionServiceTestFixture::server_ = std::make_shared<GetRandomRegionService>();
  std::thread server_thread([]() {
      rclcpp::spin(GetRandomRegionServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
