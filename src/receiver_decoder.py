#!/usr/bin/env python
import serial
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Header


class Receiver(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        #setup serial
        self.baud_rate = rospy.get_param("baud_rate",115200)
        self.port_name = rospy.get_param("port_name","/dev/lora_arduino")
        self.ser = serial.Serial(self.port_name,self.baud_rate)

        # Publications
        self.pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)

        while True:
            cmd_data = self.ser.readline()
            data = cmd_data.split(" ")
            print(data)
            '''
            if len(data)==8:
                print(data)
                odom = Odometry()
                odom.pose.pose.position.x = (float(data[0]))/100.
                odom.pose.pose.position.y = (float(data[1]))/100.
                odom.pose.pose.position.z = (float(data[2]))/100.
                odom.pose.pose.orientation.x = (float(data[3]))/100.
                odom.pose.pose.orientation.y = (float(data[4]))/100.
                odom.pose.pose.orientation.z = (float(data[5]))/100.
                odom.pose.pose.orientation.w = (float(data[6]))/100.
                odom.header = Header()
                self.pub_odom.publish(odom)
                '''

    def on_shutdown(self):
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("lora_receiver",anonymous=False)
    transmission = Receiver()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()