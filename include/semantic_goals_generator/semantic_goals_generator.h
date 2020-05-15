/*
 * SEMANTIC GOALS GENERATOR ROS NODE
 *
 * Author: Alberto José Tudela Roldán
 * Copyright (c) 2020, Universidad de Málaga
 * All rights reserved.
 *
 */

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Universidad de Málaga nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#ifndef SEMANTICGOALSGENERATOR_H
#define SEMANTICGOALSGENERATOR_H
 
#include <cmath>
#include <random>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include "semantic_goals_generator/polygon.h"
#include "semantic_goals_generator/SemanticGoals.h"

class SemanticGoalsGenerator{
     public:
        SemanticGoalsGenerator(ros::NodeHandle& node, ros::NodeHandle& node_private);
		~SemanticGoalsGenerator();
     private:
        ros::NodeHandle node_, nodePrivate_;
        ros::Subscriber mapSub_;
        ros::Publisher navGoalsPub_, roiPub_;
        ros::ServiceServer paramsSrv_, navsGenSrv_;
        
        geometry_msgs::Pose origin_;        
        bool isCostmap_;
        int width_, height_, inflatedFootprintSize_;        
        int cellMinX_, cellMaxX_, cellMinY_, cellMaxY_;
        float bBoxMinX_, bBoxMaxX_, bBoxMinY_, bBoxMaxY_;
        float mapMinX_, mapMaxX_, mapMinY_, mapMaxY_;
        float resolution_, inflationRadius_;
        std::string mapFrame_;
        std::vector<int8_t> mapData_;       
        Polygon roi_;
        std::vector<Polygon> roiVector_;

        void initialize() { std_srvs::Empty empt; updateParams(empt.request, empt.response); }
        bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
        bool SemanticGoalsService(semantic_goals_generator::SemanticGoals::Request& req, semantic_goals_generator::SemanticGoals::Response& res);
        void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msgMap);
        std::vector<Polygon> getROIParams();
        void publishPolygonRoi();
        void processBoundingBox();
        int cell(int x, int y);
        bool inROI(float x, float y);
        bool inCollision(int x, int y);
};
#endif
