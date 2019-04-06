#!/usr/bin/env python
import serial
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Receiver(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        #setup serial
        self.baud_rate = rospy.get_param("baud_rate",115200)
        self.port_name = rospy.get_param("port_name","/dev/lora_arduino")
        self.ser = serial.Serial(self.port_name,self.baud_rate)

        # Publications
        self.pub_Odom = rospy.Publisher("odom", Od ometry, queue_size=1)

        while True:
            try:
                cmd_data = self.ser.readline()
                self.pub_odom.Publish()

    def on_shutdown(self):
        
        rospy.loginfo("shutting down [%s]" %(self.node_name))


if __name__ == "__main__":
    rospy.init_node("lora_receiver",anonymous=False)
    transmission = Receiver()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()