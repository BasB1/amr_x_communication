#!/usr/bin/env python

import pypozyx as pzx
import numpy as np
import rospy
import tf
from filterpy.kalman import KalmanFilter
import yaml
import os 
import math

class Localize(object):
    def __init__(self, pozyx, dt, ranging_protocol, robot_list, tag_pos, robot_number, alpha, noise):
        self.pozyx = pozyx
        self.ranging_protocol = ranging_protocol
        self.tag_pos = tag_pos
        self.robot_number = robot_number
        
        self.distance_1 = pzx.DeviceRange()
        self.distance_2 = pzx.DeviceRange()
        self.distance_3 = pzx.DeviceRange()
        self.distance_4 = pzx.DeviceRange()
        
        self.A = robot_list[robot_number]['left']
        self.B = robot_list[robot_number]['right']
        
        if robot_number == 1:
            self.C = robot_list[2]['left']
            self.D = robot_list[2]['right']
        elif robot_number == 2:
            self.C = robot_list[1]['left']
            self.D = robot_list[1]['right']
        
        self.distance_prev_1 = 0
        self.distance_prev_2 = 0
        self.distance_prev_3 = 0
        self.distance_prev_4 = 0
        
        self.f1 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f1.P = 1
        self.f1.H = np.array([[1.]])
        self.f1.F = np.array([[1.]])
        self.f1.B = np.array([[1.]])
        self.f1.Q = noise
        self.f1.R = 0.1
        self.f1.alpha = alpha
        
        self.f2 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f2.P = 1
        self.f2.H = np.array([[1.]])
        self.f2.F = np.array([[1.]])
        self.f2.B = np.array([[1.]])
        self.f2.Q = noise
        self.f2.R = 0.1
        self.f2.alpha = alpha
        
        self.f3 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f3.P = 1
        self.f3.H = np.array([[1.]])
        self.f3.F = np.array([[1.]])
        self.f3.B = np.array([[1.]])
        self.f3.Q = noise
        self.f3.R = 0.1
        self.f3.alpha = alpha
        
        self.f4 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f4.P = 1
        self.f4.H = np.array([[1.]])
        self.f4.F = np.array([[1.]])
        self.f4.B = np.array([[1.]])
        self.f4.Q = noise
        self.f4.R = 0.1
        self.f4.alpha = alpha
        
        self.f5 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f5.P = 1
        self.f5.H = np.array([[1.]])
        self.f5.F = np.array([[1.]])
        self.f5.B = np.array([[1.]])
        self.f5.Q = 0.1
        self.f5.R = 0.001
        self.f5.alpha = 1
        
        self.pozyx.setRangingProtocol(self.ranging_protocol)
        self.br = tf.TransformBroadcaster()
        
    def getDistances(self):
        # Distance 1 = AC
        # Distance 2 = AD
        # Distance 3 = BC
        # Distance 4 = BD
        # Where A(left) B(right) are local and C(left) D(right) are remote

        self.pozyx.doRanging(self.C, self.distance_1)
        self.pozyx.doRanging(self.D, self.distance_3)
        self.pozyx.doRanging(self.C, self.distance_2, self.B)        
        self.pozyx.doRanging(self.D, self.distance_4, self.B)
        
        if self.distance_1[1] == 0 :
            self.distance_1[1] = self.distance_prev_1
        if self.distance_2[1] == 0 :
            self.distance_2[1] = self.distance_prev_2
        if self.distance_3[1] == 0 :
            self.distance_3[1] = self.distance_prev_3
        if self.distance_4[1] == 0 :
            self.distance_4[1] = self.distance_prev_4
        
        self.distance_prev_1 = self.distance_1[1]
        self.distance_prev_2 = self.distance_2[1]
        self.distance_prev_3 = self.distance_3[1]
        self.distance_prev_4 = self.distance_4[1]
        
        self.f1.predict()
        self.f1.update(self.distance_1[1])
        self.f2.predict()
        self.f2.update(self.distance_2[1])
        self.f3.predict()
        self.f3.update(self.distance_3[1])
        self.f4.predict()
        self.f4.update(self.distance_4[1])       
        
    def triangulationLocal(self):
        r_0 = self.f1.x[0] * 0.001
        r_1 = self.f2.x[0] * 0.001      
        r_2 = self.f3.x[0] * 0.001
        r_3 = self.f4.x[0] * 0.001
        
        a = self.tag_pos[0]
        b = self.tag_pos[1]
        c = self.tag_pos[2]
        d = self.tag_pos[3]
        
        T = math.sqrt((c - a) ** 2 + (d - b) ** 2)
        D_left = (T + r_0 + r_1) * (T + r_0 - r_1) * (T - r_0  + r_1) * (-T + r_0 + r_1)
        D_right = (T + r_2 + r_3) * (T + r_2 - r_3) * (T - r_2  + r_3) * (-T + r_2 + r_3)
        
        if D_left < 0:
            D_left = D_left * -1
            
        if D_right < 0:
            D_right = D_right * -1
            
        O_left = (D_left ** 0.5) / 4
        O_right = (D_right ** 0.5) / 4

        LEFT_x_1 = ((a + b) / 2) + (((c - a) * (r_0 ** 2 - r_1 ** 2)) / (2 * T ** 2)) + 2 * ((b - d) / (T ** 2)) * O_left
        LEFT_x_2 = ((a + b) / 2) + (((c - a) * (r_0 ** 2 - r_1 ** 2)) / (2 * T ** 2)) - 2 * ((b - d) / (T ** 2)) * O_left
        LEFT_y_1 = ((b + d) / 2) + (((d - b) * (r_0 ** 2 - r_1 ** 2)) / (2 * T ** 2)) + 2 * ((a - c) / (T ** 2)) * O_left
        LEFT_y_2 = ((b + d) / 2) + (((d - b) * (r_0 ** 2 - r_1 ** 2)) / (2 * T ** 2)) - 2 * ((a - c) / (T ** 2)) * O_left

        RIGHT_x_1 = ((a + b) / 2) + (((c - a) * (r_2 ** 2 - r_3 ** 2)) / (2 * T ** 2)) + 2 * ((b - d) / (T ** 2)) * O_right
        RIGHT_x_2 = ((a + b) / 2) + (((c - a) * (r_2 ** 2 - r_3 ** 2)) / (2 * T ** 2)) - 2 * ((b - d) / (T ** 2)) * O_right
        RIGHT_y_1 = ((b + d) / 2) + (((d - b) * (r_2 ** 2 - r_3 ** 2)) / (2 * T ** 2)) + 2 * ((a - c) / (T ** 2)) * O_right
        RIGHT_y_2 = ((b + d) / 2) + (((d - b) * (r_2 ** 2 - r_3 ** 2)) / (2 * T ** 2)) - 2 * ((a - c) / (T ** 2)) * O_right
        
        ROBOT_x_1 = (LEFT_x_1 + RIGHT_x_1) / 2
        ROBOT_y_1 = (LEFT_y_1 + RIGHT_y_1) / 2
        ROBOT_x_2 = (LEFT_x_2 + RIGHT_x_2) / 2
        ROBOT_y_2 = (LEFT_y_2 + RIGHT_y_2) / 2
        
        self.f5.predict()
        
        ROBOT_w_1 = math.tan((LEFT_x_1 - RIGHT_x_1) / (LEFT_y_1 - RIGHT_y_1))
        if LEFT_y_1 < RIGHT_y_1:
            ROBOT_w_1 = math.degrees(ROBOT_w_1) + 180
        else:
            ROBOT_w_1 = math.degrees(ROBOT_w_1)
        ROBOT_w_1 = math.radians(ROBOT_w_1) * -1
        
        ROBOT_w_2 = math.tan((LEFT_x_2 - RIGHT_x_2) / (LEFT_y_2 - RIGHT_y_2))
        if LEFT_y_2 < RIGHT_y_2:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        else:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        ROBOT_w_2 = math.radians(ROBOT_w_2) * -1
        
        self.f5.update(ROBOT_w_1)
        print(ROBOT_w_1)
        
        self.br.sendTransform((LEFT_x_1, LEFT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "left_tag_1",
                     "world")
        self.br.sendTransform((LEFT_x_2, LEFT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "left_tag_2",
                     "world")
        self.br.sendTransform((RIGHT_x_1, RIGHT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "right_tag_1",
                     "world")
        self.br.sendTransform((RIGHT_x_2, RIGHT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "right_tag_2",
                     "world")
        self.br.sendTransform((ROBOT_x_1, ROBOT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, self.f5.x[0]),
                     rospy.Time.now(),
                     "robot_pos_1",
                     "world")
        self.br.sendTransform((ROBOT_x_2, ROBOT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, ROBOT_w_2),
                     rospy.Time.now(),
                     "robot_pos_2",
                     "world")
    
if __name__ == "__main__":
    rospy.init_node('pozyx_triangulation')
    
    serial_port = str(rospy.get_param('~serial_port', pzx.get_first_pozyx_serial_port()))
    
    frequency = float(rospy.get_param('~frequency', 10))
    rate = rospy.Rate(frequency)
    dt = 1000/frequency
    
    alpha = float(rospy.get_param('~alpha', 0.1))
    noise = float(rospy.get_param('~noise', 1))
    
    robot_number = rospy.get_param('~robot_number')
    
    left_tag_pos_x = float(rospy.get_param('~left_tag_pos_x'))
    left_tag_pos_y = float(rospy.get_param('~left_tag_pos_y'))
    right_tag_pos_x = float(rospy.get_param('~right_tag_pos_x'))
    right_tag_pos_y = float(rospy.get_param('~right_tag_pos_y'))
    
    tag_pos = [left_tag_pos_x, left_tag_pos_y, right_tag_pos_x, right_tag_pos_y]
    
    protocol = str(rospy.get_param('~protocol', 'fast'))  
    
    if protocol == 'fast':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_FAST
    elif protocol == 'precise':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_PRECISION
    else:
        rospy.logerr("Wrong value given for protocol. Either give: 'fast' or 'precise'")
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_FAST
        
    pozyx = pzx.PozyxSerial(serial_port)
    stream = open(os.path.dirname(os.path.realpath(__file__)) + "/robot_list.yaml", "r")
    robot_list = yaml.load(stream)
    
    r = Localize(pozyx, dt, ranging_protocol, robot_list, tag_pos, robot_number, alpha, noise)
    
    while not rospy.is_shutdown():
        r.getDistances()
        r.triangulationLocal()
        
        rate.sleep()