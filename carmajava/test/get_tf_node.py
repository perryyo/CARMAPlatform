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
class TFNode(object):
    def __init__(self):
        # Init nodes
        rospy.init_node('get_tf_node', anonymous=True)
        
        # Setup tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # Publishers
        self.result_tf_pub = rospy.Publisher('/result_tf', TransformStamped, queue_size=10)

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                trans = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("TF Exception")
                rate.sleep()
                continue
            self.result_tf_pub.publish(trans)
            rate.sleep()

if __name__ == '__main__':
    try:
        tfn = TFNode()
        tfn.run()
        # prevent python from exiting until this node is stopped
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
