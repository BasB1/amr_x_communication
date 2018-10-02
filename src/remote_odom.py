#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf

class Odom(object):
    def __init__(self, itter):
        self.odom_data = Odometry()
        self.listener = tf.TransformListener()
        self._x = float()
        self._y = float()
        self._z = float()
        self.itter = itter
        
    def getData(self, data):
        self.odom_data = data
    
    def calcZero(self):
        for i in range(self.itter):
            try:
                (trans,rot) = self.listener.lookupTransform('/robot_pos_1', '/world', rospy.Time(0))
                
                self._x += trans[0] + (self.odom_data.pose.pose.postions.x * -1)
                self._y += trans[1] + (self.odom_data.pose.pose.postions.y * -1)
                
                quaternion = (
                    self.odom_data.pose.pose.orientation.x,
                    self.odom_data.pose.pose.orientation.y,
                    self.odom_data.pose.pose.orientation.z,
                    self.odom_data.pose.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                
                self._z += euler[2] + rot[2]
                rospy.sleep(0.1 / self.itter)
            except:
                i = i - 1
        
        return self._x, self._y, self._z
        
if __name__ == "__main__":
    rospy.init_node('remote_odom')
    rate = rospy.Rate(30)
    
    itter = float(rospy.get_param('~itterations', 100.))
    
    r = Odom()
    rx_topic = 'uwb_rx'
    rospy.Subscriber(rx_topic, Odometry, r.getData)
    
    br = tf.TransformBroadcaster()
    
    x, y, z = r.calcZero(itter)
    zero_pos_x = x / itter
    zero_pos_y = y / itter
    zero_rot_z = z / itter
    
    while not rospy.is_shutdown():    
        br.sendTransform((zero_pos_x, zero_pos_y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, zero_rot_z),
                     rospy.Time.now(),
                     "zero",
                     "robot_pos_1")
        rate.sleep()