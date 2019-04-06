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

        #setup serial
        self.baud_rate = rospy.get_param("baud_rate",115200)
        self.port_name = rospy.get_param("port_name","/dev/lora_arduino")
        self.ser = serial.Serial(self.port_name,self.baud_rate)

        # Subscriptions
        self.sub_cmd_drive = rospy.Subscriber("odom",Odometry,self.cb_odom,queue_size=1)
        
    def cb_odom(self,msg):
        cmd = self.odom_to_str(msg)        
        self.ser.write(cmd)
        
    def odom_to_str(self,odom):
        cmd = ""
        cmd += str(int(odom.pose.pose.position.x*100)) + ","
        cmd += str(int(odom.pose.pose.position.y*100)) + ","
        cmd += str(int(odom.pose.pose.position.z*100)) + ","
        cmd += str(int(odom.pose.pose.orientation.x*100)) + ","
        cmd += str(int(odom.pose.pose.orientation.y*100)) + ","
        cmd += str(int(odom.pose.pose.orientation.z*100)) + ","
        cmd += str(int(odom.pose.pose.orientation.w*100))

        return str

    def on_shutdown(self):
        
        rospy.loginfo("shutting down [%s]" %(self.node_name))


if __name__ == "__main__":
    rospy.init_node("lora_sender",anonymous=False)
    transmission = Sender()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()
