#!/usr/bin/env python
import pypozyx as pzx
import rospy
import yaml
import os 
from nav_msgs.msg import Odometry
import json
from pypozyx import Data
import zlib

class Communicate(object):
    def __init__(self, pozyx, destination):
        self.destination = destination
        self.pozyx = pozyx
        self.read_data = ""
        self.odom_data = Odometry()
        self.odom_data_prev = Odometry()
        self.rx_info = pzx.RXInfo()
    
    def odomData(self, data):
        self.odom_data = data
        
    def txData(self):
        x = {
                "a": round(self.odom_data.pose.pose.position.x, 4),
                "b": round(self.odom_data.pose.pose.position.y, 4),
                "c": round(self.odom_data.pose.pose.orientation.z, 4), 
                "d": round(self.odom_data.pose.pose.orientation.w, 4),
                "e": round(self.odom_data.twist.twist.linear.x, 4),
                "f": round(self.odom_data.twist.twist.angular.z, 4)
            }
        s = json.dumps(x)
        comp_data = zlib.compress(str(s))
        rospy.loginfo(len(comp_data))
        data = Data([ord(c) for c in comp_data])
        self.pozyx.sendData(self.destination, data)
                
    def rxData(self):       
        self.pozyx.getRxInfo(self.rx_info)
        data = Data([0]*self.rx_info[1])
        self.pozyx.readRXBufferData(data)   
        message = str() 
        
        for i in data:
            message = message + chr(i)
        
        s = zlib.decompress(message)
        y = json.loads(s)
        
        odom_data_pub = Odometry()

        odom_data_pub.pose.pose.position.x = y['c']
        odom_data_pub.pose.pose.position.y = y['d']
        odom_data_pub.pose.pose.orientation.z = y['e']
        odom_data_pub.pose.pose.orientation.w = y['f']

        odom_data_pub.twist.twist.linear.x = y['g']
        odom_data_pub.twist.twist.angular.z = y['h']
        
        return odom_data_pub

def main():
    try:
        odom_data = com.rxData()
        pub.publish(odom_data)
    except Exception as e:
        pass
    com.txData()

if __name__ == "__main__":
    rospy.init_node('uwb_node')
    
    serial_port = str(rospy.get_param('~serial_port', pzx.get_first_pozyx_serial_port()))
    frequency = float(rospy.get_param('~frequency', 10))
    rate = rospy.Rate(frequency)
        
    pozyx = pzx.PozyxSerial(serial_port)
    
    destination = rospy.get_param('~destination', 0x6e2f)
    tx_topic = str(rospy.get_param('~tx_topic', 'uwb_server_tx'))
    rx_topic = str(rospy.get_param('~rx_topic', 'uwb_server_rx'))
    
    pub = rospy.Publisher(rx_topic, Odometry, queue_size = 10)
    com = Communicate(pozyx, destination)
    rospy.Subscriber(tx_topic, Odometry, com.odomData)
    
    while not rospy.is_shutdown():
        main()         
        rate.sleep()
