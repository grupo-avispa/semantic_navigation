// Copyright (c) 2020 Alberto J. Tudela Roldán
// Copyright (c) 2020 Grupo Avispa, DTE, Universidad de Málaga
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

#ifndef SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_NAVIGATION_PANEL_HPP_
#define SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_NAVIGATION_PANEL_HPP_

#include <QtWidgets>
#include <memory>

// ROS
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "semantic_navigation_msgs/srv/generate_random_goals.hpp"
#include "semantic_navigation_msgs/srv/get_region_name.hpp"

class QLineEdit;
class QPushButton;

namespace semantic_navigation_rviz_plugins
{

/**
 * @class semantic_navigation_rviz_plugins::SemanticNavigationPanel
 * @brief Panel to send the robot to a room using semantic information.
 */
class SemanticNavigationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  /**
   * @brief Constructor.
   */
  explicit SemanticNavigationPanel(QWidget * parent = 0);

  /**
   * @brief Destructor.
   */
  ~SemanticNavigationPanel() override = default;

  /**
   * @brief Initialize the panel.
   */
  void onInitialize() override;

  /**
   * @brief Load the configuration of the panel.
   */
  void load(const rviz_common::Config & config) override;

  /**
   * @brief Save the configuration of the panel.
   */
  void save(rviz_common::Config config) const override;

public Q_SLOTS:
  /**
   * @brief Generate goals to send the robot to the room.
   */
  void set_room_name(const QString & name);

protected Q_SLOTS:
  /**
   * @brief Generate goals to send the robot to the room.
   */
  void generate_goals();

  /**
   * @brief Request the location of the robot.
   */
  void request_room();

  /**
   * @brief Update the room name.
   */
  void update_room_name();

protected:
  using GenerateRandomGoals = semantic_navigation_msgs::srv::GenerateRandomGoals;
  using GetRegionName = semantic_navigation_msgs::srv::GetRegionName;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Send the robot to a pose.
   *
   * @param pose Pose to send the robot.
   */
  void navigate_to_pose(geometry_msgs::msg::PoseStamped pose);

  /**
   * @brief Callback to get the response of the goal.
   *
   * @param goal_handle Handle of the goal.
   */
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);

  /**
   * @brief Callback to get the result of the goal.
   *
   * @param result Result of the goal.
   */
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Client<GenerateRandomGoals>::SharedPtr goals_generator_client_;
  rclcpp::Client<GetRegionName>::SharedPtr region_name_client_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  QLineEdit * room_name_editor_;
  QString room_name_;
  QPushButton * navigate_button_;
  QPushButton * request_room_button_;
};

}  // namespace semantic_navigation_rviz_plugins

#endif  // SEMANTIC_NAVIGATION_RVIZ_PLUGINS__SEMANTIC_NAVIGATION_PANEL_HPP_
