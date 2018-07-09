#!/usr/bin/env python
# license removed for brevity
#
# ROS Node converts a mobility request or path message from a bag file to have an updated time stamp
# This causes that message to appear as if it had just been received
#
import time
import rospy
import tf2_ros

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix

# Function to return current time
current_time_millis = lambda: long(round(time.time() * 1000))

MPS_PER_MPH = 0.44704

# Node Class
class TimeCorrectorNode(object):
    def __init__(self):
        # Init nodes
        rospy.init_node('get_tf_node', anonymous=True)
        
        # Setup tf listener
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        # Publishers
        self.result_tf_pub = rospy.Publisher('/result_tf', TransformStamped, queue_size=10)

        # Subscribers
        self.gps_sub = rospy.Subscriber("/saxton_cav/drivers/sensor_fusion/filtered/nav_sat_fix", NavSatFix, self.gps_cb)
        self.object_sub = rospy.Subscriber("/saxton_cav/drivers/sensor_fusion/filtered/nav_sat_fix", NavSatFix, self.gps_cb)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(turtle_name, 'turtle1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue


    # Function to nav sat fix messages
    def gps_cb(self, msg):

        try:
            trans = tfBuffer.lookup_transform('earth', 'host_vehicle', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        result_msg = TransformStamped()

        self.result_tf_pub.publish(result_msg)

    # Function to handle path messages from bag file
    def path_message_cb(self, path):

        path.header.sender_id = "time_corrector_node"
        path.header.timestamp = current_time_millis()

        self.path_pub.publish(path)

if __name__ == '__main__':
    try:
        tcn = TimeCorrectorNode()
        # prevent python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
