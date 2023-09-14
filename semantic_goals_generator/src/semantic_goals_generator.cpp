/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Copyright (c) 2020-2023 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

// C++
#include <limits>

// ROS
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "slg_msgs/point2D.hpp"
#include "polygon_utils/polygon_utils.hpp"
#include <yaml-cpp/yaml.h>

// Semantic Goals
#include "semantic_goals_generator/semantic_goals_generator.hpp"

/* Initialize the subscribers and publishers */
SemanticGoalsGenerator::SemanticGoalsGenerator(): Node("semantic_goals_generator"), border_(0.0), 
	direction_(SemanticGoals::Request::RANDOM){
	// Initialize ROS parameters
	get_params();

	// Publishers
	goals_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
		"semantic_goals", rclcpp::QoS(1).transient_local());
	polygon_viz_pub_ = this->create_publisher<polygon_msgs::msg::Polygon2DCollection>(
		"polygons", rclcpp::QoS(1).transient_local());
	names_viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"names", rclcpp::QoS(1).transient_local());

	// Subscribers
	map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
		map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
		std::bind(&SemanticGoalsGenerator::map_callback, this, std::placeholders::_1));

	// Services
	using std::placeholders::_1, std::placeholders::_2;
	goals_generator_service_ = this->create_service<SemanticGoals>(
		"semantic_goals", 
		std::bind(&SemanticGoalsGenerator::goals_generator_service, this, _1, _2));
	semantic_position_service_ = this->create_service<SemanticPosition>(
		"semantic_position", 
		std::bind(&SemanticGoalsGenerator::semantic_position_service, this, _1, _2));

	show_visualization();
}

/* Update parameters of the node */
void SemanticGoalsGenerator::get_params(){
	// BOOLEAN PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "is_costmap", 
		rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Is the map a costmap?"));
	this->get_parameter("is_costmap", is_costmap_);
	RCLCPP_INFO(this->get_logger(), "The parameter is_costmap is set to: [%s]", 
		is_costmap_ ? "true" : "false");

	nav2_util::declare_parameter_if_not_declared(this, "full_map", 
		rclcpp::ParameterValue(false), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Use the full map as ROI?"));
	this->get_parameter("full_map", full_map_);
	RCLCPP_INFO(this->get_logger(), "The parameter full_map is set to: [%s]", 
		full_map_ ? "true" : "false");
	
	// FLOAT PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "inflation_radius", 
		rclcpp::ParameterValue(0.5), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Inflation radius for the robot footprint")
			.set__floating_point_range({rcl_interfaces::msg::FloatingPointRange()
			.set__from_value(0.0).set__to_value(10.0).set__step(0.01)}));
	this->get_parameter("inflation_radius", inflation_radius_);
	RCLCPP_INFO(this->get_logger(), "The parameter inflation_radius is set to: [%f]", 
		inflation_radius_);

	// STRING PARAMS ..........................................................................
	nav2_util::declare_parameter_if_not_declared(this, "map_topic", 
		rclcpp::ParameterValue("map"), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("Name of the map topic"));
	this->get_parameter("map_topic", map_topic_);
	RCLCPP_INFO(this->get_logger(), "The parameter map_topic is set to: [%s]", 
		map_topic_.c_str());

	std::string rois_filename;
	nav2_util::declare_parameter_if_not_declared(this, "rois_filename", 
		rclcpp::ParameterValue("rois.yaml"), rcl_interfaces::msg::ParameterDescriptor()
			.set__description("File where the ROIs are stored"));
	this->get_parameter("rois_filename", rois_filename);
	RCLCPP_INFO(this->get_logger(), "The parameter rois_filename is set to: [%s]", 
		rois_filename.c_str());

	get_roi_params(rois_filename);
	if (roi_list_.empty()){
		RCLCPP_ERROR(this->get_logger(), "The list of ROIs could not be found");
		exit(1);
	}
}

/* Get rois from YAML file */
void SemanticGoalsGenerator::get_roi_params(const std::string &filename){
	RCLCPP_INFO(this->get_logger(), "Reading ROIs from file: %s", filename.c_str());
	YAML::Node config = YAML::LoadFile(filename);

	// Get the list of ROIs
	if (config["rois"]){
		for (const auto& roi : config["rois"]) {
			ROI new_roi;
			// Extract name and yaw
			new_roi.yaw = roi["yaw"].as<float>();
			new_roi.set_name(roi["name"].as<std::string>());
			// Extract edges
			for (const auto& edge : roi["edges"]) {
				slg::Point2D a(edge[0][0].as<float>(), edge[0][1].as<float>());
				slg::Point2D b(edge[1][0].as<float>(), edge[1][1].as<float>());
				new_roi.polygon.add_edge({a,b});
			}
			roi_list_.push_back(new_roi);
		}
	}else{
		RCLCPP_ERROR(this->get_logger(), "No ROIs found in file [%s]", filename.c_str());
	}
}

/* Map callback */
void SemanticGoalsGenerator::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
	std::lock_guard<std::recursive_mutex> cfl(mutex_);
	RCLCPP_INFO(this->get_logger(), "Received a %d X %d map @ %.3f m/pix", 
		msg->info.width, msg->info.height, msg->info.resolution);

	map_ = *msg;

	map_min_x_ = map_.info.origin.position.x;
	map_max_x_ = map_.info.origin.position.x + map_.info.width * map_.info.resolution;
	map_min_y_ = map_.info.origin.position.y;
	map_max_y_ = map_.info.origin.position.y + map_.info.height * map_.info.resolution;
}

/* Calculate bounding box for ROI */
void SemanticGoalsGenerator::process_boundingbox(ROI roi){
	// Region of interest (ROI) must lie inside the map boundaries
	// If ROI is empty, the whole map is treated as ROI by default
	if (roi.empty()){
		bbox_min_x_ = map_min_x_;
		bbox_max_x_ = map_max_x_;
		bbox_min_y_ = map_min_y_;
		bbox_max_y_ = map_max_y_;
		RCLCPP_INFO(this->get_logger(), "No ROI specified, full map is used");
	}else{
		// If the ROI is outside the map, adjust to map boundaries
		// Determine bounding box of ROI
		bbox_min_x_ = std::numeric_limits<double>::infinity();
		bbox_max_x_ = -std::numeric_limits<double>::infinity();
		bbox_min_y_ = std::numeric_limits<double>::infinity();
		bbox_max_y_ = -std::numeric_limits<double>::infinity();

		slg::Polygon polygon_roi = roi.polygon;
		for (int e = 0; e < polygon_roi.size(); e++){
			slg::Point2D p = polygon_roi.get_edge(e).a;

			if (p.x < map_min_x_) p.x = map_min_x_;
			if (p.x > map_max_x_) p.x = map_max_x_;

			if (p.x < bbox_min_x_) bbox_min_x_ = p.x;
			if (p.x > bbox_max_x_) bbox_max_x_ = p.x;

			if (p.y < map_min_y_) p.y = map_min_y_;
			if (p.y > map_max_y_) p.y = map_max_y_;

			if (p.y < bbox_min_y_) bbox_min_y_ = p.y;
			if (p.y > bbox_max_y_) bbox_max_y_ = p.y;
		}
	}

	// Calculate bounding box for cell array
	cell_min_x_ = static_cast<int>((bbox_min_x_ - map_.info.origin.position.x) / 
		map_.info.resolution);
	cell_max_x_ = static_cast<int>((bbox_max_x_ - map_.info.origin.position.x) / 
		map_.info.resolution);
	cell_min_y_ = static_cast<int>((bbox_min_y_ - map_.info.origin.position.y) / 
		map_.info.resolution);
	cell_max_y_ = static_cast<int>((bbox_max_y_ - map_.info.origin.position.y) / 
		map_.info.resolution);

	RCLCPP_INFO(this->get_logger(), "ROI bounding box (meters): (%f,%f) (%f,%f)", 
		bbox_min_x_, bbox_min_y_, bbox_max_x_, bbox_max_y_);
	RCLCPP_INFO(this->get_logger(), "ROI bounding box (cells): (%i,%i) (%i,%i)", 
		cell_min_x_, cell_min_y_, cell_max_x_, cell_max_y_);
}

/* Service for sending random goals based on labeled rois */
bool SemanticGoalsGenerator::goals_generator_service(
	const std::shared_ptr<SemanticGoals::Request> request, 
	std::shared_ptr<SemanticGoals::Response> response){
	
	std::lock_guard<std::recursive_mutex> cfl(mutex_);
	ROI current_roi;
	RCLCPP_INFO(this->get_logger(), "Incoming service request: %i, %s", 
		request->n, request->roi_name.c_str());

	// Get arguments
	int n = request->n;
	for (const auto& roi: roi_list_){
		if (roi.polygon.get_name() == request->roi_name){
			current_roi = roi;
			break;
		}
	}
	direction_ = request->direction;
	border_ = request->border;

	// If the requested ROI is empty and we don't want to use the full map
	if (current_roi.empty() && !full_map_){
		RCLCPP_FATAL(this->get_logger(), "The requested ROI [%s], could not be found in the list", 
			request->roi_name.c_str());
		return false;
	}

	// Check if we have a map
	if (map_.data.empty()) {
		RCLCPP_FATAL(this->get_logger(), "Failed to get map at [%s]", map_topic_.c_str());
		return false;
	}

	inflated_footprint_size_ = static_cast<int>(inflation_radius_ / map_.info.resolution) + 1;

	// Process bounding box
	process_boundingbox(current_roi);

	// Generate response
	response->goals.header.frame_id = map_topic_;

	// Generate random goal pose
	std::random_device rd; // obtain a random number from hardware
	std::mt19937 gen(rd()); // seed the generator
	std::uniform_int_distribution<int> dist_x(cell_min_x_, cell_max_x_); // define the range
	std::uniform_int_distribution<int> dist_y(cell_min_y_, cell_max_y_); // define the range
	std::uniform_real_distribution<double> dist_pi(0.0, 2 * M_PI);

	int count = 0;
	while( (response->goals.poses.size() < n)){
		count += 1;
		int cell_x = dist_x(gen);
		int cell_y = dist_y(gen);

		geometry_msgs::msg::Pose pose;
		pose.position.x = cell_x * map_.info.resolution + map_.info.origin.position.x;
		pose.position.y = cell_y * map_.info.resolution + map_.info.origin.position.y;

		// If the point lies within ROI and is not in collision
		if (current_roi.in_roi(pose.position.x, pose.position.y) && 
			!in_collision(cell_x, cell_y) && 
			current_roi.distance_from_borders(pose.position.x, pose.position.y, border_)){
			// Generate orientation
			double yaw;
			if (direction_ == SemanticGoals::Request::OUTSIDE){
				yaw = atan2((pose.position.y - current_roi.polygon.centroid().y), 
					(pose.position.x - current_roi.polygon.centroid().x));
			}else if (direction_ == SemanticGoals::Request::INSIDE){
				yaw = atan2((pose.position.y - current_roi.polygon.centroid().y), 
					(pose.position.x - current_roi.polygon.centroid().x)) + M_PI;
			}else if (direction_ == SemanticGoals::Request::STORED){
				if (current_roi.yaw > -M_PI && current_roi.yaw < M_PI){
					yaw = current_roi.yaw;
				}else{
					yaw = dist_pi(gen);
				}
			}else if (direction_ == SemanticGoals::Request::REQUESTED){
				if (request->yaw > -M_PI && request->yaw < M_PI){
					yaw = request->yaw;
				}else{
					yaw = dist_pi(gen);
				}
			}else{
				yaw = dist_pi(gen);
			}
			pose.orientation = tf2::toMsg(tf2::Quaternion({0, 0, 1}, yaw));
			RCLCPP_INFO(this->get_logger(), "Pose %i (x: %f, y: %f, yaw: %f)", 
				response->goals.poses.size()+1, pose.position.x, pose.position.y, yaw);

			response->goals.poses.push_back(pose);
		}
	}

	goals_pub_->publish(response->goals);
	return true;
}

/* Service for request the semantic pose  */
bool SemanticGoalsGenerator::semantic_position_service(
	const std::shared_ptr<SemanticPosition::Request> request, 
	std::shared_ptr<SemanticPosition::Response> response){

	RCLCPP_INFO(this->get_logger(), "Incoming service request: [%f, %f]", 
		request->position.x, request->position.y);

	// Get arguments and check if the point lies within ROI
	for (auto& roi: roi_list_){
		if (roi.in_roi(request->position.x, request->position.y)){
			response->roi_name = roi.get_name();
			return true;
		}
	}

	response->roi_name = SemanticPosition::Response::UNKNOWN;
	RCLCPP_FATAL(this->get_logger(), "Failed to get semantic position");
	return false;
}

/* Return the cell of the costmap */
int8_t SemanticGoalsGenerator::cell(int x, int y){
	// Return 'unknown' if out of bounds
	if (x < 0 || y < 0 || x >= map_.info.width  || y >= map_.info.height){
		return nav2_util::OCC_GRID_UNKNOWN;
	}

	return map_.data[x +  map_.info.width * y];
}

/* Check if a point is in collision */
bool SemanticGoalsGenerator::in_collision(int x, int y){
	int x_min, x_max, y_min, y_max;

	if (is_costmap_){
		if (cell(x, y) != nav2_util::OCC_GRID_FREE) return true;
		return false;
	}

	x_min = x - inflated_footprint_size_;
	x_max = x + inflated_footprint_size_;
	y_min = y - inflated_footprint_size_;
	y_max = y + inflated_footprint_size_;

	for (int i = x_min; i < x_max; i++){
		for (int j = y_min; j < y_max; j++){
			if (cell(i, j) != nav2_util::OCC_GRID_FREE) return true;
		}
	}
	return false;
}

/* Show the rois in rviz */
void SemanticGoalsGenerator::show_visualization(){
	polygon_msgs::msg::Polygon2DCollection polygon_array;
	polygon_array.header.frame_id = map_topic_;
	polygon_array.header.stamp = this->now();

	visualization_msgs::msg::MarkerArray names_array;

	for (auto & roi: roi_list_){
		// Push the polygon
		polygon_array.polygons.push_back(polygon_utils::polygon3Dto2D(roi.polygon));

		// Create label
		visualization_msgs::msg::Marker label_marker;
		label_marker.header.frame_id = map_topic_;
		label_marker.header.stamp = this->now();
		label_marker.ns = "labelroi";
		label_marker.id = polygon_array.polygons.size();
		label_marker.text = roi.get_name();
		label_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
		label_marker.action = visualization_msgs::msg::Marker::ADD;
		label_marker.pose.position.x = roi.polygon.centroid().x;
		label_marker.pose.position.y = roi.polygon.centroid().y;
		label_marker.pose.position.z = 0.05;
		label_marker.pose.orientation.x = 0.0;
		label_marker.pose.orientation.y = 0.0;
		label_marker.pose.orientation.z = 0.0;
		label_marker.pose.orientation.w = 1.0;
		label_marker.scale.z = 0.5;
		label_marker.color.r = 1.0;
		label_marker.color.g = 1.0;
		label_marker.color.b = 1.0;
		label_marker.color.a = 1.0f;
		names_array.markers.push_back(label_marker);
	}

	polygon_viz_pub_->publish(polygon_array);
	names_viz_pub_->publish(names_array);
}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = std::make_shared<SemanticGoalsGenerator>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
