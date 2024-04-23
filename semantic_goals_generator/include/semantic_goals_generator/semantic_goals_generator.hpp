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

#ifndef SEMANTIC_NAVIGATION__SEMANTIC_GOALS_GENERATOR_HPP_
#define SEMANTIC_NAVIGATION__SEMANTIC_GOALS_GENERATOR_HPP_

// C++
#include <cmath>
#include <mutex>
#include <random>
#include <string>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "polygon_msgs/msg/polygon2_d_collection.hpp"
#include "slg_msgs/polygon.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "semantic_navigation_msgs/srv/semantic_goals.hpp"
#include "semantic_navigation_msgs/srv/semantic_position.hpp"
#include "semantic_navigation_msgs/srv/semantic_regions.hpp"

struct ROI{
	slg::Polygon polygon;
	float yaw;

	inline bool empty(){ return polygon.empty(); };
	inline void clear(){ return polygon.clear(); };
	inline std::string get_name(){ return polygon.get_name(); };
	inline void set_name(std::string name){ polygon.set_name(name); };

	/* Check if a point is inside the region of interest (ROI) */
	bool in_roi(float x, float y){
		if (polygon.size() == 0) return true;
		return polygon.contains(slg::Point2D(x, y));
	}

	/* Check if the point is at distance from all borders */
	bool distance_from_borders(float x, float y, float border){
		for (auto& edge: polygon.get_edges()){
			if (edge.distance(slg::Point2D(x,y)) < border) return false;
		}
		return true;
	}
};

class SemanticGoalsGenerator : public rclcpp::Node{
	public:
		SemanticGoalsGenerator();
	private:
		using SemanticGoals = semantic_navigation_msgs::srv::SemanticGoals;
		using SemanticPosition = semantic_navigation_msgs::srv::SemanticPosition;
		using SemanticRegions = semantic_navigation_msgs::srv::SemanticRegions;

		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goals_pub_;
		rclcpp::Publisher<polygon_msgs::msg::Polygon2DCollection>::SharedPtr polygons_viz_pub_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr names_viz_pub_;
		rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

		rclcpp::Service<SemanticGoals>::SharedPtr goals_generator_service_;
		rclcpp::Service<SemanticPosition>::SharedPtr semantic_position_service_;
		rclcpp::Service<SemanticRegions>::SharedPtr semantic_regions_service_;

		std::recursive_mutex mutex_;
		nav_msgs::msg::OccupancyGrid map_;
		bool is_costmap_, full_map_;
		int inflated_footprint_size_;
		int cell_min_x_, cell_max_x_, cell_min_y_, cell_max_y_;
		float bbox_min_x_, bbox_max_x_, bbox_min_y_, bbox_max_y_;
		float map_min_x_, map_max_x_, map_min_y_, map_max_y_;
		float inflation_radius_, border_;
		std::string goals_topic_, polygons_topic_, names_topic_, map_topic_;
		std::string direction_;
		std::vector<ROI> roi_list_;

		void get_params();
		void get_roi_params(const std::string &filename);
		bool goals_generator_service(const std::shared_ptr<SemanticGoals::Request> request,
			std::shared_ptr<SemanticGoals::Response> response);
		bool semantic_position_service(const std::shared_ptr<SemanticPosition::Request> request,
			std::shared_ptr<SemanticPosition::Response> response);
		bool semantic_regions_service(const std::shared_ptr<SemanticRegions::Request> request,
			std::shared_ptr<SemanticRegions::Response> response);
		void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
		
		void show_visualization();
		void process_boundingbox(ROI roi);
		int8_t cell(unsigned int x, unsigned int y);
		bool in_collision(int x, int y);
};

#endif // SEMANTIC_NAVIGATION__SEMANTIC_GOALS_GENERATOR_HPP_
