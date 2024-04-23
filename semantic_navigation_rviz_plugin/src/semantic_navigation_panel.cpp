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

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "rviz_common/display_context.hpp"
#include "nav2_util/robot_utils.hpp"
#include "semantic_navigation_rviz_plugin/semantic_navigation_panel.hpp"

namespace semantic_navigation_rviz_plugin{

semanticNavigationPanel::semanticNavigationPanel(QWidget* parent) : Panel(parent){
	// We lay out the "room name" text entry field using a QLabel and a QLineEdit in a QHBoxLayout.
	QHBoxLayout* room_name_layout = new QHBoxLayout;
	room_name_layout->addWidget(new QLabel("Room name:"));
	room_name_editor_ = new QLineEdit;
	room_name_layout->addWidget(room_name_editor_);

	// Lay out the buttons in a QHBoxLayout 
	QHBoxLayout* buttons_layout = new QHBoxLayout;
	navigate_button_ = new QPushButton("Send the robot to the room");
	buttons_layout->addWidget(navigate_button_);
	navigate_button_->setEnabled(false);

	request_room_button_ = new QPushButton("Where is the robot?");
	buttons_layout->addWidget(request_room_button_);
	request_room_button_->setEnabled(true);

	// Lay out the room name field and the buttons
	QVBoxLayout* layout = new QVBoxLayout;
	layout->addLayout(room_name_layout);
	layout->addLayout(buttons_layout);
	setLayout(layout);

	// Conect buttons with functions
	connect(navigate_button_, SIGNAL(clicked()), this, SLOT(generate_goals()));
	connect(request_room_button_, SIGNAL(clicked()), this, SLOT(request_room()));
	connect(room_name_editor_, SIGNAL(textChanged(QString)), this, SLOT(update_room_name()));
}

void semanticNavigationPanel::onInitialize(){
	auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
	ros_node_ = lock->get_raw_node();

	// Service clients
	goals_generator_client_ = ros_node_->create_client<SemanticGoals>("semantic_goals");
	semantic_position_client_ = ros_node_->create_client<SemanticPosition>("semantic_position");

	// Initialize transform buffer and listener
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/* Read the room name from the QLineEdit and call set_room_name() with the results.
 * This is connected to QLineEdit::editingFinished() which fires when the user presses
 * Enter or Tab or otherwise moves focus away.
 */
void semanticNavigationPanel::update_room_name(){
	set_room_name(room_name_editor_->text());
}

/* Set the topic name we are publishing to. */
void semanticNavigationPanel::set_room_name(const QString& name){
	// Only take action if the name has changed.
	if(name != room_name_){
		room_name_ = name;
		// rviz::Panel defines the configChanged() signal.  Emitting it
		// tells RViz that something in this panel has changed that will
		// affect a saved config file.  Ultimately this signal can cause
		// QWidget::setWindowModified(true) to be called on the top-level
		// rviz::VisualizationFrame, which causes a little asterisk ("*")
		// to show in the window's title bar indicating unsaved changes.
		Q_EMIT configChanged();
	}

	// Gray out the buttons when the name is empty.
	navigate_button_->setEnabled(room_name_ != "");
	request_room_button_->setEnabled(room_name_ == "");
}

/* Save all configuration data from this panel to the given
 * Config object.  It is important here that you call save()
 * on the parent class so the class id and panel name get saved.
 */
void semanticNavigationPanel::save(rviz_common::Config config)const{
	rviz_common::Panel::save(config);
	config.mapSetValue("Room", room_name_);
}

/* Load all configuration data for this panel from the given Config object. */
void semanticNavigationPanel::load(const rviz_common::Config& config){
	rviz_common::Panel::load(config);
	QString rName;

	if (config.mapGetString("Room", &rName)){
		room_name_editor_->setText(rName);
		update_room_name();
	}
}

/* Generate goals */
void semanticNavigationPanel::generate_goals(){
	while (!goals_generator_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(ros_node_->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(ros_node_->get_logger(), "Service not available, waiting again...");
	}

	// Send the request and wait for the response
	geometry_msgs::msg::PoseArray goals;
	auto request = std::make_shared<SemanticGoals::Request>();
	request->n = 1;
	request->roi_name = room_name_.toStdString();
	request->direction = semantic_navigation_msgs::srv::SemanticGoals::Request::INSIDE;
	request->border = 0.1;
	auto result = goals_generator_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticGoals>::SharedFuture future){
			if (future.get()->goals.poses.size() > 0){
				// Send goals to the navigation stack
				geometry_msgs::msg::PoseStamped goal;
				goal.header.frame_id = "map";
				goal.header.stamp = ros_node_->now();
				goal.pose = future.get()->goals.poses[0];
				navigate_to_pose(goal);
				room_name_editor_->setText("");
			}else{
				room_name_editor_->setText("Couldn't send the goal.");
			}
	});

	update_room_name();
}

/* Request the location of the robot */
void semanticNavigationPanel::request_room(){
	// Transform the robot position to the map frame
	geometry_msgs::msg::PoseStamped robot_pose;
	if (!nav2_util::getCurrentPose(robot_pose, *tf_buffer_, "map", "base_link", 0.5)){
		room_name_editor_->setText("I don't know where the robot is.");
		update_room_name();
		return;
	}

	while (!semantic_position_client_->wait_for_service(std::chrono::seconds(1))) {
		if (!rclcpp::ok()) {
			RCLCPP_ERROR(ros_node_->get_logger(), 
				"Interrupted while waiting for the service. Exiting.");
			return;
		}
		RCLCPP_INFO(ros_node_->get_logger(), "Service not available, waiting again...");
	}

	// Send the request and wait for the response
	auto request = std::make_shared<SemanticPosition::Request>();
	request->position.x = robot_pose.pose.position.x;
	request->position.y = robot_pose.pose.position.y;
	auto result = semantic_position_client_->async_send_request(request, 
		[this](rclcpp::Client<SemanticPosition>::SharedFuture future){
			if (future.get()->roi_name != 
				semantic_navigation_msgs::srv::SemanticPosition::Response::UNKNOWN){
				room_name_ = QString::fromStdString(future.get()->roi_name);
				room_name_editor_->setText(room_name_);
			}else{
				room_name_editor_->setText("I don't know where the robot is.");
			}
	});
	update_room_name();
}

void semanticNavigationPanel::navigate_to_pose(geometry_msgs::msg::PoseStamped pose){
	// Create the action client that will send the goal to the navigation stack.
	navigation_client_ = rclcpp_action::create_client<NavigateToPose>(ros_node_, "navigate_to_pose");
	if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1))) {
		RCLCPP_ERROR(ros_node_->get_logger(), "Action server not available after waiting");
		return;
	}

	// Populate a goal message
	auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
	goal_msg.pose = pose;
	goal_msg.pose.header.frame_id = "map";
	goal_msg.pose.header.stamp = ros_node_->now();

	// Send the goal
	RCLCPP_INFO(ros_node_->get_logger(), "Sending goal request");
	auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
	send_goal_options.goal_response_callback =
		std::bind(&semanticNavigationPanel::goal_response_callback, this, std::placeholders::_1);
	send_goal_options.result_callback =
		std::bind(&semanticNavigationPanel::result_callback, this, std::placeholders::_1);

	auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);
}

void semanticNavigationPanel::goal_response_callback(
	const GoalHandleNavigateToPose::SharedPtr & goal_handle){
	if (!goal_handle){
		RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server");
	}else{
		RCLCPP_INFO(ros_node_->get_logger(), "Goal accepted by server, waiting for result");
	}
}

void semanticNavigationPanel::result_callback(
	const GoalHandleNavigateToPose::WrappedResult & result){
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			RCLCPP_INFO(ros_node_->get_logger(), "Goal was reached");
			break;
		case rclcpp_action::ResultCode::ABORTED:
			RCLCPP_ERROR(ros_node_->get_logger(), "Goal was failed");
			break;
		case rclcpp_action::ResultCode::CANCELED:
			RCLCPP_ERROR(ros_node_->get_logger(), "Goal was canceled");
			break;
		default:
			RCLCPP_ERROR(ros_node_->get_logger(), "Unknown result code");
			break;
	}
}

}  // end namespace 

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(semantic_navigation_rviz_plugin::semanticNavigationPanel, rviz_common::Panel)
