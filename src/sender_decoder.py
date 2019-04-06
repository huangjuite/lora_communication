#!/usr/bin/env python
import serial
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Sender(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Publications
        self.pub_motor_cmd = rospy.Publisher("motor_cmd", Motor4Cmd, queue_size=1)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("cmd_drive",VelocityVector,self.cbCmd,queue_size=1)
        
    def on_shutdown(self):
        
        rospy.loginfo("shutting down [%s]" %(self.node_name))


if __name__ == "__main__":
    rospy.init_node("lora_sender",anonymous=False)
    transmission = Sender()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()
