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

#include <stdio.h>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

#include "semanticNavigationPanel.h"

namespace semantic_navigation_rviz_plugin{

semanticNavigationPanel::semanticNavigationPanel(QWidget* parent){
	// We lay out the "room name" text entry field using a QLabel and a QLineEdit in a QHBoxLayout.
	QHBoxLayout* roomNameLayout = new QHBoxLayout;
	roomNameLayout->addWidget(new QLabel("Room name:"));
	roomNameEditor_ = new QLineEdit;
	roomNameLayout->addWidget(roomNameEditor_);

	// Lay out the buttons in a QHBoxLayout 
	QHBoxLayout* buttonsLayout = new QHBoxLayout;
	sendGoalButton_ = new QPushButton("Send the robot to the room");
	buttonsLayout->addWidget(sendGoalButton_);
	sendGoalButton_->setEnabled(false);

	requestRoomButton_ = new QPushButton("Where is the robot?");
	buttonsLayout->addWidget(requestRoomButton_);
	requestRoomButton_->setEnabled(true);

	// Lay out the room name field and the buttons
	QVBoxLayout* layout = new QVBoxLayout;
	layout->addLayout(roomNameLayout);
	layout->addLayout(buttonsLayout);
	setLayout(layout);

	// Conect buttons with functions
	connect(sendGoalButton_, SIGNAL(clicked()), this, SLOT(sendGoal()));
	connect(requestRoomButton_, SIGNAL(clicked()), this, SLOT(requestRoom()));
	connect(roomNameEditor_, SIGNAL(textChanged(QString)), this, SLOT(updateRoomName()));

	// Service clients
	clientSemanticGoals_ = node_.serviceClient<semantic_goals_generator::SemanticGoals>("semantic_goals");
	clientSemanticPos_ = node_.serviceClient<semantic_goals_generator::SemanticPosition>("semantic_position");

	// Publisher
	pubGoal_ = node_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
}

semanticNavigationPanel::~semanticNavigationPanel(){
}

/* Read the room name from the QLineEdit and call setRoomName() with the results.
 * This is connected to QLineEdit::editingFinished() which fires when the user presses
 *  Enter or Tab or otherwise moves focus away. */
void semanticNavigationPanel::updateRoomName(){
	setRoomName(roomNameEditor_->text());
}

/* Set the topic name we are publishing to. */
void semanticNavigationPanel::setRoomName(const QString& name){
	// Only take action if the name has changed.
	if(name != roomName_){
		roomName_ = name;
		// rviz::Panel defines the configChanged() signal.  Emitting it
		// tells RViz that something in this panel has changed that will
		// affect a saved config file.  Ultimately this signal can cause
		// QWidget::setWindowModified(true) to be called on the top-level
		// rviz::VisualizationFrame, which causes a little asterisk ("*")
		// to show in the window's title bar indicating unsaved changes.
		Q_EMIT configChanged();
	}

	// Gray out the buttons when the name is empty.
	sendGoalButton_->setEnabled(roomName_ != "");
	requestRoomButton_->setEnabled(roomName_ == "");
}

/* Save all configuration data from this panel to the given
   Config object.  It is important here that you call save()
   on the parent class so the class id and panel name get saved. */
void semanticNavigationPanel::save(rviz::Config config)const{
	rviz::Panel::save(config);
	config.mapSetValue("Room", roomName_);
}

/* Load all configuration data for this panel from the given Config object. */
void semanticNavigationPanel::load(const rviz::Config& config){
	rviz::Panel::load(config);
	QString rName;

	if(config.mapGetString("Room", &rName)){
		roomNameEditor_->setText(rName);
		updateRoomName();
	}
}

/* Send Goal */
void semanticNavigationPanel::sendGoal(){
	semantic_goals_generator::SemanticGoals srvSemanticGoals;
	geometry_msgs::PoseArray goals;
	int nGoals = 1;
	float border = 0.1;

	srvSemanticGoals.request.n = nGoals;
	srvSemanticGoals.request.roi_name = roomName_.toStdString();
	srvSemanticGoals.request.direction = semantic_goals_generator::SemanticGoalsRequest::INSIDE;
	srvSemanticGoals.request.border = border;

	if(clientSemanticGoals_.call(srvSemanticGoals)){
		goals = srvSemanticGoals.response.goals;

		for(int g = 0; g < goals.poses.size(); g++){
			geometry_msgs::PoseStamped goal;
			goal.header = goals.header;
			goal.pose = goals.poses[g];
			pubGoal_.publish(goal);
		}
		roomNameEditor_->setText("");
		updateRoomName();
	}else{
		roomNameEditor_->setText("Couldn't send the goal.");
		updateRoomName();
	}
}

/* Request the location of the robot */
void semanticNavigationPanel::requestRoom(){
	semantic_goals_generator::SemanticPosition srvSemanticPosition;
	geometry_msgs::Point pos;

	tf::StampedTransform transform;
	try{
		tfListener_.lookupTransform("map","base_link",ros::Time(0), transform);
		pos.x = transform.getOrigin().x();
		pos.y = transform.getOrigin().y();
	}catch(const tf::TransformException& ex) {
		ROS_ERROR("Nope! %s", ex.what());
		roomNameEditor_->setText("I don't know where the robot is.");
		updateRoomName();
		return;
	}

	srvSemanticPosition.request.position = pos;

	if(clientSemanticPos_.call(srvSemanticPosition)){
		roomName_ = QString::fromStdString(srvSemanticPosition.response.roi_name);
		roomNameEditor_->setText(roomName_);
		updateRoomName();
	}else{
		roomNameEditor_->setText("I don't know where the robot is.");
		updateRoomName();
	}
}

} // end namespace 

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(semantic_navigation_rviz_plugin::semanticNavigationPanel, rviz::Panel)

