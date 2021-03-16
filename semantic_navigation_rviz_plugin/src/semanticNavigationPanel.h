/*
 * SEMANTIC NAVIGATION RVIZ PLUGIN
 *
 * Copyright (c) 2020-2021 Alberto José Tudela Roldán <ajtudela@gmail.com>
 * 
 * This file is part of semantic_navigation.
 * 
 * All rights reserved.
 *
 */

#ifndef SEMANTIC_NAVIGATION_PANEL_H
#define SEMANTIC_NAVIGATION_PANEL_H

#include "ros/ros.h"
#include <rviz/panel.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

#include <semantic_goals_generator/SemanticGoals.h>
#include <semantic_goals_generator/SemanticPosition.h>

#include <QtWidgets>

class QLineEdit;

namespace semantic_navigation_rviz_plugin{

class semanticNavigationPanel: public rviz::Panel{
Q_OBJECT
	public:
		semanticNavigationPanel(QWidget* parent = 0);
		~semanticNavigationPanel();

		virtual void load(const rviz::Config& config);
		virtual void save(rviz::Config config) const;

	public Q_SLOTS:
		void setRoomName(const QString& name);

	protected Q_SLOTS:
		void sendGoal();
		void requestRoom();
		void updateRoomName();

	protected:
		ros::NodeHandle node_;
		ros::ServiceClient clientSemanticGoals_, clientSemanticPos_;
		ros::Publisher pubGoal_;
		tf::TransformListener tfListener_;
		
		QLineEdit* roomNameEditor_;
		QString roomName_;
		QPushButton* sendGoalButton_;
		QPushButton* requestRoomButton_;
};

} // end namespace 

#endif 
