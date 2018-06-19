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

#include "wgs84_utils.h"

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
    TransformMaintainer(tf2_ros::Buffer& tf2_buffer, tf2_ros::TransformBroadcaster& tf2_broadcaster,
        std::unordered_map<std::string, nav_msgs::OdometryConstPtr> odom_map,
        std::unordered_map<std::string, sensor_msgs::NavSatFixConstPtr> nav_sat_map,
        std::unordered_map<std::string, cav_msgs::HeadingStampedConstPtr> heading_map)
    {
        tf2_buffer_ = tf2_buffer;
        tf2_broadcaster_ = tf2_broadcaster;
    }

    void heading_update_cb();

    void nav_sat_fix_update_cb();

    void odometry_update_cb();

    // void heading_update_cb(const ros::MessageEvent<cav_msgs::HeadingStamped>& event);

    // void nav_sat_fix_update_cb(const ros::MessageEvent<sensor_msgs::NavSatFix>& event);

    // void odometry_update_cb(const ros::MessageEvent<nav_msgs::Odometry>& event);

private:

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

      // Host vehicle state variables
    bool heading_received_ = false;
    bool nav_sat_fix_received_ = false;
    wgs84_utils::wgs84_coordinate host_veh_loc_;
    double host_veh_heading_;

    std::unordered_map<std::string, nav_msgs::OdometryConstPtr> odom_map_;
    std::unordered_map<std::string, sensor_msgs::NavSatFixConstPtr> navsatfix_map_;
    std::unordered_map<std::string, cav_msgs::HeadingStampedConstPtr> heading_map_;
    
    // The heading of the vehicle in degrees east of north in an NED frame.
    // Frame ids
    const std::string earth_frame_;
    const std::string map_frame_;
    const std::string odom_frame_;
    const std::string base_link_frame_;
    const std::string global_pos_sensor_frame_;
    const std::string local_pos_sensor_frame_;

    // Transforms
    tf2::Transform map_to_odom_ = tf2::Transform::getIdentity();
    tf2::Transform earth_to_map_;
    tf2::Transform base_to_local_pos_sensor_;
    tf2::Transform base_to_global_pos_sensor_;
    tf2::Transform odom_to_base_link_ = tf2::Transform::getIdentity();// The odom frame will start in the same orientation as the base_link frame on startup
    // Transform Update parameters
    ros::Time prev_map_time_;
    ros::Duration MAP_UPDATE_PERIOD; // 5.0 TODO Time in seconds between updating the map frame location
    int tf_sequence_count_ = 0;
    bool first_transform_ = true;
};

