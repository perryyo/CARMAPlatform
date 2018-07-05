// Test truck_message_definitions

#include "../src/transform_maintainer.h"

#include <array>
#include <gtest/gtest.h>

// /**
//    * Tests the handling of nav sat fix messages
//    * Also tests the updating of the earth->map transform and map->odom transform
//    * This test is dependant on the handleHeading and handleOdometry tests passing
//    * @throws Exception
//    */
//   @Test
//   public void testHandleNavSatFix() throws Exception {
//     MockRoadwayManager roadwayMgr = new MockRoadwayManager();
//     TransformMaintainer envWkr = new TransformMaintainer(roadwayMgr, log, "earth", "map", "odom",
//       "base_link", "pinpoint", "pinpoint");

//     // Publish the transform from base_link to position sensor
//     Transform baseToPositionSensor = Transform.identity();
//     TFMessage tfMsg = messageFactory.newFromType(TFMessage._TYPE);
//     geometry_msgs.Transform baseToPositionSensorMsg = messageFactory.newFromType(geometry_msgs.Transform._TYPE);
//     baseToPositionSensorMsg = baseToPositionSensor.toTransformMessage(baseToPositionSensorMsg);
//     geometry_msgs.TransformStamped tfStamped = messageFactory.newFromType(TransformStamped._TYPE);
//     tfStamped.setChildFrameId(envWkr.globalPositionSensorFrame);
//     tfStamped.getHeader().setFrameId(envWkr.baseLinkFrame);
//     tfStamped.setTransform(baseToPositionSensorMsg);
//     tfMsg.setTransforms(Arrays.asList(tfStamped));
//     roadwayMgr.publishTF(tfMsg);

//     // Initial heading message
//     HeadingStamped headingMsg = messageFactory.newFromType(HeadingStamped._TYPE);
//     float heading = 90;
//     headingMsg.setHeading(heading);
//     envWkr.handleHeadingMsg(headingMsg);
//     // Location at prime meridian and equator
//     NavSatFix navMsg = messageFactory.newFromType(NavSatFix._TYPE);
//     navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//     navMsg.setLatitude(0);
//     navMsg.setLongitude(0);
//     navMsg.setAltitude(0);
//     navMsg.getHeader().setFrameId(envWkr.globalPositionSensorFrame);
//     envWkr.handleNavSatFixMsg(navMsg);

//     assertTrue(envWkr.navSatFixReceived);
//     assertTrue(envWkr.hostVehicleLocation.almostEqual(new Location(0,0,0), 0.00001, 0.0000001));

//     // First time calling nav sat fix so check earth to map transform (should be an NED frame at starting location)
//     GeodesicCartesianConverter gcc = new GeodesicCartesianConverter();
//     Transform earthToMap = gcc.ecefToNEDFromLocaton(envWkr.hostVehicleLocation);
//     // an NED at lat = 0 lon = 0 is rotated -90 deg around the ecef y-axis
//     Vector3 solutionTrans = new Vector3(6378137.0, 0, 0);
//     Vector3 solRotAxis = new Vector3(0,1,0);
//     Quaternion solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(-90));
//     assertTrue(earthToMap.getTranslation().almostEquals(solutionTrans, 1.0));// Check accuracy to within 1m
//     assertTrue(earthToMap.getRotationAndScale().almostEquals(solutionRot, 0.0001)); // Check accuracy to within ~0.01 deg

//     // The heading of 90 degrees means
//     // the actual orientation of odom when there has been no odometry is with +x being due east
//     solutionTrans = new Vector3(0.0, 0, 0);
//     solRotAxis = new Vector3(0,0,1);
//     solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(90));
//     assertTrue(envWkr.mapToOdom.getTranslation().almostEquals(solutionTrans, 0.00001));
//     assertTrue(envWkr.mapToOdom.getRotationAndScale().almostEquals(solutionRot, 0.0001));

//     // Odometry moves the vehicle 10 meters along north axis
//     // Build odometry message
//     Odometry odometryMsg = messageFactory.newFromType(Odometry._TYPE);
//     PoseWithCovariance pose = odometryMsg.getPose();
//     pose.getPose().getPosition().setX(10); // 10 m north
//     pose.getPose().getPosition().setY(0);
//     pose.getPose().getPosition().setZ(0);
//     geometry_msgs.Quaternion quatMsg = pose.getPose().getOrientation();
//     pose.getPose().setOrientation(Quaternion.identity().toQuaternionMessage(quatMsg));
//     odometryMsg.getHeader().setFrameId(envWkr.odomFrame);
//     odometryMsg.setChildFrameId(envWkr.localPositionSensorFrame);

//     // Call function
//     envWkr.handleOdometryMsg(odometryMsg);

//     // Find the new lat lon which will be used to compare
//     Point3D realLocInMap = new Point3D(1,10,0);
//     Location realLoc = gcc.cartesian2Geodesic(realLocInMap, earthToMap);
//     // Send new lat lon to EnvironmentWorker
//     navMsg = messageFactory.newFromType(NavSatFix._TYPE);
//     navMsg.getStatus().setStatus(NavSatStatus.STATUS_FIX);
//     navMsg.setLatitude(realLoc.getLatitude());
//     navMsg.setLongitude(realLoc.getLongitude());
//     navMsg.setAltitude(realLoc.getAltitude());
//     navMsg.getHeader().setFrameId(envWkr.globalPositionSensorFrame);
//     envWkr.handleNavSatFixMsg(navMsg);

//     // Map earth transform should be unchanged
//     assertTrue(earthToMap.almostEquals(envWkr.earthToMap, 0.0000001));
//     // Compare new mapToOdom
//     // The odom frame should be moved +1 along the map's +x axis
//     solutionTrans = new Vector3(1, 0, 0);
//     solRotAxis = new Vector3(0,0,1);
//     solutionRot = Quaternion.fromAxisAngle(solRotAxis, Math.toRadians(90));
//     assertTrue(envWkr.mapToOdom.getTranslation().almostEquals(solutionTrans, 0.00001));
//     assertTrue(envWkr.mapToOdom.getRotationAndScale().almostEquals(solutionRot, 0.0001));

bool equal_vector3(tf2::Vector3 v1, tf2::Vector3 v2, double epsilon) {
  return 
    fabs(v1.getX() - v2.getX()) < epsilon &&
    fabs(v1.getY() - v2.getY()) < epsilon &&
    fabs(v1.getZ() - v2.getZ()) < epsilon;
}  

bool equal_quaternion(tf2::Quaternion v1, tf2::Quaternion v2, double epsilon) {
  return 
    equal_vector3(v1.getAxis(), v2.getAxis(), epsilon) &&
    fabs(v1.getW() - v2.getW()) < epsilon;
}   
// TODO
TEST(TruckMessageParsingTest, test1)
{

  // Set the transform from base_link to position sensor
  tf2::Transform base_to_global_pos_sensor = tf2::Transform::getIdentity();

  // Location at prime meridian and equator with 90 deg initial heading
  wgs84_utils::wgs84_coordinate host_veh_coord;
  host_veh_coord.lat = 0.0;
  host_veh_coord.lon = 0.0;
  host_veh_coord.elevation = 0.0;
  host_veh_coord.heading = 90.0 * wgs84_utils::DEG2RAD;

  // an NED at lat = 0 lon = 0 is rotated -90 deg around the ecef y-axis
  tf2::Vector3 earth_to_map_trans(6378137.0, 0, 0);
  tf2::Vector3 y_axis(0,1,0);
  tf2::Quaternion earth_to_map_rot(y_axis, -90.0 * wgs84_utils::DEG2RAD);
  tf2::Transform earth_to_map(earth_to_map_rot, earth_to_map_trans);

  // Vehicle has not moved some transform from odom to base link should be identity
  tf2::Transform odom_to_base_link = tf2::Transform::getIdentity();

  tf2::Transform map_to_odom = TransformMaintainer::calculate_map_to_odom_tf(
    host_veh_coord,
    base_to_global_pos_sensor, 
    earth_to_map,
    odom_to_base_link
  );

  // The heading of 90 degrees means
  // the actual orientation of odom when there has been no odometry is with +x being due east
  tf2::Vector3 solution_trans(0.0, 0, 0);
  tf2::Vector3 sol_rot_axis(0,0,1);
  tf2::Quaternion solution_rot = tf2::Quaternion(sol_rot_axis, 90 * wgs84_utils::DEG2RAD);

  EXPECT_EQ(true, equal_vector3(map_to_odom.getOrigin(), solution_trans, 0.00001));
  EXPECT_EQ(true, equal_quaternion(map_to_odom.getRotation(), solution_rot, 0.0001));

// EXPECT_FLOAT_EQ(0.25 - 4000, propb_12.getWrenchEffortEcho());
}

int main(int argc, char**argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}