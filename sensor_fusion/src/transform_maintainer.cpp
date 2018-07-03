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

#include <std::string>
#include <math.h>
#include <unordered_map>
#include <memory>

#include "transform_maintainer.h"

void TransformMaintainer::heading_update_cb()
{
  // TODO Might not be needed
  host_vehicle_heading_ = event.getMessage().get()->;
  heading_received_ = true;
}

void TransformMaintainer::nav_sat_fix_update_cb()
{
    // Assign the new host vehicle location
    if (navsatfix_map_.empty() || heading_map_.empty()) {
      std::string msg = "TransformMaintainer nav_sat_fix_update_cb called before heading and nav_sat_fix recieved";
      log.warn("TRANSFORM", msg);// TODO
      return; // If we don't have a heading and a nav sat fix the map->odom transform cannot be calculated
    }
    sensor_msgs::NavSatFixConstPtr host_veh_loc = navsatfix_map_.begin()->second;
    
    std::string frame_id = host_veh_loc.getHeader().getFrameId();
    if (frame_id != global_pos_sensor_frame_) {
      std::string msg = "NavSatFix message with unsupported frame received. Frame: " + frameId;
      log.fatal("TRANSFORM", msg);// TODO
      return;
    }
    
    // Check if base_link->position_sensor tf is available. If not look it up
    if (base_to_global_pos_sensor_ == null) {
      // This transform should be static. No need to look up more than once
      base_to_global_pos_sensor_ = get_transform(base_link_frame_, global_pos_sensor_frame_, ros::Time(0));
      // TODO wrap with try catch
      // return if exception
    }

    vector<geometry_msgs::TransformStamped> tf_stamped_msgs;

    // Update map location on start
    if (first_transform_) { 
      // Map will be an NED frame on the current vehicle location
      earth_to_map_ = ecef_to_ned_from_loc(host_veh_loc.latitude, host_veh_loc.longitude, host_veh_loc.altitude);
      first_transform_ = false;
    }
    // Keep publishing transform to maintain timestamp
    geometry_msgs::TransformStamped earth_to_map_msg;
    void tf2::transformTF2ToMsg (earth_to_map_, earth_to_map_msg, host_veh_loc.getHeader().getStamp(), earth_frame_, map_frame_);	
    tf_stamped_msgs.push_back(earth_to_map_msg);

    // Calculate map->global_position_sensor transform

    wgs84_coordinate host_veh_coord;
    host_veh_coord.lat = host_veh_loc.latitude;
    host_veh_coord.lon = host_veh_loc.longitude;
    host_veh_coord.elevation = host_veh_loc.altitude;

    tf2::Vector3 global_sensor_in_map = wgs84_utils::geodesic_2_cartesian(host_veh_coord, earth_to_map_.invert());

    // T_x_y = transform describing location of y with respect to x
    // m = map frame
    // b = baselink frame (from odometry)
    // B = baselink frame (from nav sat fix)
    // o = odom frame
    // p = global position sensor frame
    // We want to find T_m_o. This is the new transform from map to odom.
    // T_m_o = T_m_B * inv(T_o_b)  since b and B are both odom.
    tf2::Vector3 sensor_trans_in_map = global_sensor_in_map.getOrigin();
    // The vehicle heading is relative to NED so over short distances heading in NED = heading in map
    tf2::Vector3 zAxis = tf2::Vector3(0, 0, 1);
    Quaternion sensor_rot_in_map = tf2::Quaternion(zAxis, heading_map_.begin()->second * DEG2RAD);
    sensor_rot_in_map = sensor_rot_in_map.normalize();

    tf2::Transform T_m_p = tf2::Transform(sensor_trans_in_map, sensor_rot_in_map);
    tf2::Transform T_B_p = base_to_global_pos_sensor_;
    tf2::Transform T_m_B = T_m_p * T_B_p.invert();
    tf2::Transform T_o_b = odom_to_base_link_;

    // Modify map to odom with the difference from the expected and real sensor positions
    map_to_odom_ = T_m_B * T_o_b.invert();
    // Publish newly calculated transforms
    geometry_msgs::TransformStamped map_to_odom_msg;
    tf2::transformTF2ToMsg(map_to_odom_, map_to_odom_msg,  host_veh_loc.getHeader().getStamp(), map_frame_, odom_frame_);
    
    tf_stamped_msgs.push_back(map_to_odom_msg);

    // Publish transform
    tf2_broadcaster_.sendTransform(tf_stamped_msgs);
}

void TransformMaintainer::odometry_update_cb() 
{
  // TODO
}

// Helper function
tf2::Transform TransformMaintainer::get_transform(std::string parent_frame, std::string child_frame, ros::Time stamp) {
  geometry_msgs::TransformStamped transform_stamped;
  if(tf2_buffer_.canTransform(parent_frame, child_frame, stamp))
  {
    transform_stamped = tf2_buffer_.lookupTransform(parent_frame, child_frame, stamp);
  }
  else if(tf2_buffer_.canTransform(parent_frame, child_frame, ros::Time(0)))
  {
    ROS_DEBUG_STREAM("Using latest transform available");
    transform_stamped = tf2_buffer_.lookupTransform(parent_frame, child_frame, ros::Time(0));
  }
  else
  {
    ROS_WARN_STREAM("No transform available from " << parent_frame << " to " << child_frame);
    //TODO throw exception
  }

  return tf2::fromMsg(transform_stamped);
}

// TODO
// Lat and lon must be in radians
tf2::Transform TransformMaintainer::ecef_to_ned_from_loc(double lat, double lon, double elev) {

    wgs84_coordinate loc;
    loc.lat = lat;
    loc.lon = lon;
    loc.elevation = elev;
  // TODO need to think about this
    tf2::Transform ecef_point = wgs84_utils::geodesic_2_cartesian(loc, tf2::Transform.getIdentity());

    tf2::Vector3 trans = tf2::Vector3(locInECEF.getX(), locInECEF.getY(), locInECEF.getZ());

    // Rotation matrix of north east down frame with respect to ecef
    // Found at https://en.wikipedia.org/wiki/North_east_down
    double sinLat = sin(lat);
    double sinLon = sin(lon);
    double cosLat = cos(lat);
    double cosLon = cos(lon);

 	  tf2::Matrix3x3 rotMat(
      -sinLat * cosLon, -sinLon,  -cosLat * cosLon,
      -sinLat * sinLon,  cosLon,  -cosLat * sinLon,
                cosLat,       0,           -sinLat 
    );
    
    return tf2::Transform(rotMat, trans);
  }
}


