/*
 *  Copyright (C) 2017 Ongun Kanat <ongun.kanat@gmail.com>
 *  Copyright (C) 2017 Arda İnceoğlu <93arda@gmail.com>
 *  Copyright (C) 2017 Istanbul Technical University
 *                     Artificial Intelligence and Robotics Laboratory
 *                     <air.cs.itu.edu.tr>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
  * @file violet_main.h
  * @authors Ongun Kanat <ongun.kanat@gmail.com>
  * @authors Arda İnceoğlu <93arda@gmail.com>
  * @brief Implementation of event loop in Violet
  */
#ifndef VIOLET_MAIN_H
#define VIOLET_MAIN_H
#define _USE_MATH_DEFINES

/* Standard Libs */
#include <string>
#include <map>
#include <vector>
/* ROS Libs */
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
/* Messages and services */
#include <std_srvs/Empty.h>
#include <violet_msgs/DetectionInfo.h>
#include <violet_srvs/RegisterSource.h>
/* Package's headers */
#include "common.h"
#include "util.h"
#include "input_source_manager.h"
#include "knowledge_base.h"
#include "predicate_manager.h"

namespace violet {
/**
 * @brief Implementation of main loop
 */
class VioletMain
{
protected:
    /**
     * @brief Global node handle
     */
    ros::NodeHandle node_handle;
    /**
     * @brief Local node handle (/violet by default)
     */
    ros::NodeHandle local_node_handle;

    /**
     * @brief Namespace
     */
    std::string violet_ns;

    /**
     * @brief subscribe_source_srv Source registration service
     */
    ros::ServiceServer subscribe_source_srv;
    /**
     * @brief sourceRegistrationCallback The ROS callback for  source registration service calls
     * @param req Request
     * @param res Response
     * @return According to ROS conventions true for success, false for otherwise
     */
    bool sourceRegistrationCallback(violet_srvs::RegisterSource::Request &req, violet_srvs::RegisterSource::Response &res );
    /**
     * @brief subscribers List of ROS Subscribers created via input source registration
     */
    std::vector<ros::Subscriber*> subscribers;

    /**
     * @brief ROS service server for pausing addition or removal of the objects in the scene.
     */
    ros::ServiceServer add_remove_pause_srv;
    bool addRemovePauseCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /**
     * @brief world_state_pub The publisher for violet_msgs/WorldState
     */
    ros::Publisher world_state_pub;

    /**
     * @brief kb Pointer to current KnowledgeBase instance.
     *
     * It holds a pointer to current KnowledgeBase. If the package compiled with -DWITH_CLOSED_WORLD_KB option
     * it is initialized to use Closed World KB.
     */
    KnowledgeBase *kb;

    /**
     * @brief Confusion matrix used for decreasing the probabilities of the objects
     */
    boost::array<boost::array<double, 2>,2> confidence_confusion;
public:
    /**
     * @brief SceneInterpreter The constructor for SceneInterpreter class
     *
     * It initializes the subscribers, predicates, node handles and finally the KnowledgeBase
     */
    VioletMain();
    ~VioletMain();

    /**
     * @brief objectRoutineCleaning Drops probability of the objects and removes when probability drops enough
     */
    void objectRoutineCleaning();

    /**
     * @brief publishWorldState Publishes World State Message
     */
    void publishWorldState();

    /**
     * @brief updateKnowledgeBaseGraph
     */
    void updateKnowledgeBaseGraph();

#ifdef CLOSED_WORLD_KB
    void updateClosedWorldKnowledgeBase();
#endif

    /**
     * @brief print_world_state
     */
    void printWorldState();

};
} /* END OF NAMESPACE violet */

#endif
