#pragma once

/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include "object_tracker.h"
#include "twist_history_buffer.h"
#include <sensor_fusion/SensorFusionConfig.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_msgs/HeadingStamped.h>
#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bondcpp/bond.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <memory>


/**
 * TODO
 * @brief A ROS node that monitors multiple sources to produce a filtered version
 *
 * This class monitors all drivers that provide position and tracked objects api*
 *
 */
class TransformMaintainer
{
public:

    /**
     * TODO
     * @brief Initializes the ROS context of this node
     * @param argc Command line argument count
     * @param argv Command line arguments
     * @param name Name of the node
     */
    TransformMaintainer(tf2_ros::Buffer& tf2_buffer, tf2_ros::TransformBroadcaster& tf2_broadcaster)
    {
        tf2_buffer_ = tf2_buffer;
        tf2_broadcaster_ = tf2_broadcaster;
    }

    void nav_sat_fix_update_cb();

    void odometry_update_cb();

private:

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

      // Host vehicle state variables
    bool headingReceived = false;
    bool navSatFixReceived = false;
    Location hostVehicleLocation;
    double hostVehicleHeading;
    // The heading of the vehicle in degrees east of north in an NED frame.
    // Frame ids
    const std::string earthFrame;
    const std::string mapFrame;
    const std::string odomFrame;
    const std::string baseLinkFrame;
    const std::string globalPositionSensorFrame;
    const std::string localPositionSensorFrame;

    // Transforms
    Transform mapToOdom = Transform.identity();
    Transform earthToMap = null;
    Transform baseToLocalPositionSensor = null;
    Transform baseToGlobalPositionSensor = null;
    Transform odomToBaseLink = Transform.identity();// The odom frame will start in the same orientation as the base_link frame on startup
    // Transform Update parameters
    Time prevMapTime = null;
    Duration MAP_UPDATE_PERIOD = new Duration(5); // TODO Time in seconds between updating the map frame location
    int tfSequenceCount = 0;
};

