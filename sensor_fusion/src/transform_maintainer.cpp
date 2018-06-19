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
    if (first_transform_) { // TODO Determine earth->map update method || 0 < roadwayMgr.getTime().subtract(prevMapTime).compareTo(MAP_UPDATE_PERIOD)) {
      // Map will be an NED frame on the current vehicle location
      earthToMap = gcc.ecefToNEDFromLocaton(hostVehicleLocation);
      prevMapTime = roadwayMgr.getTime();
      first_transform_ = false;
    }
    // Keep publishing transform to maintain timestamp
    tfStampedMsgs.add(buildTFStamped(earthToMap, earthFrame, mapFrame, roadwayMgr.getTime()));

    // Calculate map->global_position_sensor transform
    Point3D globalSensorInMap = gcc.geodesic2Cartesian(hostVehicleLocation, earthToMap.invert());

    // T_x_y = transform describing location of y with respect to x
    // m = map frame
    // b = baselink frame (from odometry)
    // B = baselink frame (from nav sat fix)
    // o = odom frame
    // p = global position sensor frame
    // We want to find T_m_o. This is the new transform from map to odom.
    // T_m_o = T_m_B * inv(T_o_b)  since b and B are both odom.
    Vector3 nTranslation = new Vector3(globalSensorInMap.getX(), globalSensorInMap.getY(), globalSensorInMap.getZ());
    // The vehicle heading is relative to NED so over short distances heading in NED = heading in map
    Vector3 zAxis = new Vector3(0, 0, 1);
    Quaternion globalSensorRotInMap = Quaternion.fromAxisAngle(zAxis, Math.toRadians(hostVehicleHeading));
    globalSensorRotInMap = globalSensorRotInMap.normalize();

    Transform T_m_p = new Transform(nTranslation, globalSensorRotInMap);
    Transform T_B_p = baseToGlobalPositionSensor;
    Transform T_m_B = T_m_p.multiply(T_B_p.invert());
    Transform T_o_b = odomToBaseLink;

    // Modify map to odom with the difference from the expected and real sensor positions
    mapToOdom = T_m_B.multiply(T_o_b.invert());
    // Publish newly calculated transforms
    tfStampedMsgs.add(buildTFStamped(mapToOdom, mapFrame, odomFrame, roadwayMgr.getTime()));
    publishTF(tfStampedMsgs);
}

void TransformMaintainer::odometry_update_cb() 
{

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

tf2::Transform TransformMaintainer::ecef_to_ned(double lat, double lon, double elev) {

    wgs84_coordinate loc;
    loc.lat = lat;
    loc.lon = lon;
    loc.elevation = elev;
  // TODO need to think about this
    tf2::Transform ecef_point = wgs84_utils::geodesic_2_cartesian(loc, tf2::Transform.getIdentity());

    Vector3 trans = new Vector3(locInECEF.getX(), locInECEF.getY(), locInECEF.getZ());

    // Rotation matrix of north east down frame with respect to ecef
    // Found at https://en.wikipedia.org/wiki/North_east_down
    double sinLat = Math.sin(loc.getLatRad());
    double sinLon = Math.sin(loc.getLonRad());
    double cosLat = Math.cos(loc.getLatRad());
    double cosLon = Math.cos(loc.getLonRad());
    double[][] R = new double[][] {
      { -sinLat * cosLon, -sinLon,  -cosLat * cosLon },
      { -sinLat * sinLon,  cosLon,  -cosLat * sinLon },
      {           cosLat,       0,           -sinLat }
    };

    Quaternion quat = QuaternionUtils.matToQuaternion(R);
    return new Transform(trans, quat);
  }
}


