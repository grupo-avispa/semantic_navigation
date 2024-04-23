// Copyright (c) 2020 Alberto J. Tudela Roldán
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

#ifndef SEMANTIC_NAVIGATION__SEMANTIC_NAVIGATION_PANEL_HPP_
#define SEMANTIC_NAVIGATION__SEMANTIC_NAVIGATION_PANEL_HPP_

// QT
#include <QtWidgets>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "semantic_navigation_msgs/srv/semantic_goals.hpp"
#include "semantic_navigation_msgs/srv/semantic_position.hpp"

class QLineEdit;
class QPushButton;

namespace semantic_navigation_rviz_plugin{

class semanticNavigationPanel: public rviz_common::Panel{
Q_OBJECT
	public:
		explicit semanticNavigationPanel(QWidget* parent = 0);
		~semanticNavigationPanel() override = default;
		void onInitialize() override;
		void load(const rviz_common::Config& config) override;
		void save(rviz_common::Config config) const override;

	public Q_SLOTS:
		void set_room_name(const QString& name);

	protected Q_SLOTS:
		void generate_goals();
		void request_room();
		void update_room_name();

	protected:
		using SemanticGoals = semantic_navigation_msgs::srv::SemanticGoals;
		using SemanticPosition = semantic_navigation_msgs::srv::SemanticPosition;
		using NavigateToPose = nav2_msgs::action::NavigateToPose;
		using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
		
		rclcpp::Node::SharedPtr ros_node_;
		rclcpp::Client<SemanticGoals>::SharedPtr goals_generator_client_;
		rclcpp::Client<SemanticPosition>::SharedPtr semantic_position_client_;

		rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
		std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
		std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
		
		QLineEdit* room_name_editor_;
		QString room_name_;
		QPushButton* navigate_button_;
		QPushButton* request_room_button_;

		void navigate_to_pose(geometry_msgs::msg::PoseStamped pose);
		void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
		void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);
};

}  // namespace semantic_navigation_rviz_plugin

#endif 
