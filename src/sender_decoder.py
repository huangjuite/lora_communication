#!/usr/bin/env python
import serial
import struct
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
        cmd = self.odom_to_bytearray(msg)        
        self.ser.write(cmd)
        
    def odom_to_bytearray(self,odom):
        ba = bytearray(struct.pack("f",odom.pose.pose.position.x)).append(bytes("\20"))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.position.y)).append(bytes("\20")))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.position.z)).append(bytes("\20")))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.orientation.x)).append(bytes("\20")))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.orientation.y)).append(bytes("\20")))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.orientation.z)).append(bytes("\20")))
        ba.append(bytearray(struct.pack("f",odom.pose.pose.orientation.w)).append(bytes("\20")))

        return ba

    def on_shutdown(self):
        
        rospy.loginfo("shutting down [%s]" %(self.node_name))


if __name__ == "__main__":
    rospy.init_node("lora_sender",anonymous=False)
    transmission = Sender()
    rospy.on_shutdown(transmission.on_shutdown)
    rospy.spin()
