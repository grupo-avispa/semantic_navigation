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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "nav2_behavior_tree/test/utils/test_service.hpp"
#include "semantic_navigation_bt/action/generate_random_goals_service.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"

class GenerateRandomGoalsService
  : public TestService<semantic_navigation_msgs::srv::GenerateRandomGoals>
{
public:
  GenerateRandomGoalsService()
  : TestService("generate_random_goals")
  {
  }

  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<semantic_navigation_msgs::srv::GenerateRandomGoals::Request> request,
    const std::shared_ptr<semantic_navigation_msgs::srv::GenerateRandomGoals::Response> response)
  {
    (void)request_header;
    (void)request;
    response->goals.goals.push_back(geometry_msgs::msg::PoseStamped());
  }
};

class GenerateRandomGoalsServiceTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("generate_random_goals_test_fixture");
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

    factory_->registerNodeType<semantic_navigation_bt::GenerateRandomGoalsService>(
      "GenerateRandomGoals");
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
  }

  void TearDown() override
  {
    tree_.reset();
  }

  static std::shared_ptr<GenerateRandomGoalsService> server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
};

rclcpp::Node::SharedPtr GenerateRandomGoalsServiceTestFixture::node_ = nullptr;
std::shared_ptr<GenerateRandomGoalsService> GenerateRandomGoalsServiceTestFixture::server_ =
  nullptr;
BT::NodeConfiguration * GenerateRandomGoalsServiceTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> GenerateRandomGoalsServiceTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> GenerateRandomGoalsServiceTestFixture::tree_ = nullptr;

TEST_F(GenerateRandomGoalsServiceTestFixture, test_tick)
{
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <GenerateRandomGoals service_name="generate_random_goals" number_of_goals="1" region_name="region_1" yaw="0.0" orientation="outside" border="0.0" goals="{goals}"/>
        </BehaviorTree>
      </root>)";

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("service_name"), "generate_random_goals");
  EXPECT_EQ(tree_->rootNode()->getInput<int>("number_of_goals"), 1);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("region_name"), "region_1");
  EXPECT_EQ(tree_->rootNode()->getInput<float>("yaw"), 0.0);
  EXPECT_EQ(tree_->rootNode()->getInput<std::string>("orientation"), "outside");
  EXPECT_EQ(tree_->rootNode()->getInput<float>("border"), 0.0);
  EXPECT_EQ(tree_->rootNode()->executeTick(), BT::NodeStatus::SUCCESS);

  // Check if the output is correct
  auto goals = config_->blackboard->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
  EXPECT_EQ(goals.size(), 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  // initialize service and spin on new thread
  GenerateRandomGoalsServiceTestFixture::server_ = std::make_shared<GenerateRandomGoalsService>();
  std::thread server_thread([]() {
      rclcpp::spin(GenerateRandomGoalsServiceTestFixture::server_);
    });

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();
  server_thread.join();

  std::cout << "All tests passed: " << all_successful << std::endl;

  return all_successful;
}
