#!/usr/bin/env python
import roslib
import rospy
import tf2_ros
from tf2_msgs.msg import TFMessage

rospy.init_node("tf_restamper")
tfpublisher= rospy.Publisher("tf",TFMessage,queue_size=10)

def tfcallback(tfmessage):
    for transf in tfmessage.transforms:
        transf.header.stamp=rospy.Time.now()
    tfpublisher.publish(tfmessage)

tfproxy = rospy.Subscriber("tf_old",TFMessage,tfcallback)
rospy.spin()