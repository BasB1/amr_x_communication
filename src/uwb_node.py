#!/usr/bin/env python
import pypozyx as pzx
import numpy as np
import rospy
import tf
from filterpy.kalman import KalmanFilter
import yaml
import os 
import math
from nav_msgs.msg import Odometry
import json
from pypozyx import Data
import zlib

class Transform(object):
    def __init__(self, link_to_robot, tf_prefix):
         self.broadcaster = tf.TransformBroadcaster()
         self.listener = tf.TransformListener()
         self.link_to_robot = link_to_robot
         self.odom_data = Odometry()
         self.tf_prefix = tf_prefix
         
    def getTransformData(self):
        (self.trans, self.rot) = self.listener.lookupTransform(self.tf_prefix + '/odom', self.tf_prefix + '/robot_pos_1', rospy.Time(0))
      
    def odomData(self):
        self.odom_data = com.returnRxOdom()
    
    def checkDistance(self):
        (trans, rot) = self.listener.lookupTransform(self.tf_prefix + '/odom', self.tf_prefix + '/robot_pos_1', rospy.Time(0))
        return (trans[0] * trans[0] + trans[1] * trans[1]) ** 0.5
        
    def calcZero(self):
        self._x = self.trans[0] + (self.odom_data.pose.pose.position.x)
        self._y = self.trans[1] + (self.odom_data.pose.pose.position.y)
        quat = tf.transformations.quaternion_from_euler(0, 0, self.rot[2])
        
        self._quaternion = (
            0,
            0,
            self.odom_data.pose.pose.orientation.z + quat[2],
            self.odom_data.pose.pose.orientation.w + quat[3])
        
    def publishCF(self):
        self.broadcaster.sendTransform((self._x, self._y, 0),
                     (0,
                      0,
                      self._quaternion[2],
                      self._quaternion[3]),
                     rospy.Time.now(),
                     self.tf_prefix + '/zero',
                     self.tf_prefix + '/odom')
    
    def publishOF(self):
        self.broadcaster.sendTransform((self.odom_data.pose.pose.orientation.x, self.odom_data.pose.pose.orientation.y, 0),
                     (self.odom_data.pose.pose.orientation.x,
                      self.odom_data.pose.pose.orientation.y,
                      self.odom_data.pose.pose.orientation.z,
                      self.odom_data.pose.pose.orientation.w),
                     rospy.Time.now(),
                     self.tf_prefix + "/external_odom",
                     self.tf_prefix + "/zero")

class Localize(object):
    def __init__(self, pozyx, dt, ranging_protocol, robot_list, tag_pos, robot_number, alpha, noise, R, link_to_robot, do_ranging, tf_prefix):
        self.pozyx = pozyx
        self.ranging_protocol = ranging_protocol
        self.tag_pos = tag_pos
        self.robot_number = robot_number
        self.link_to_robot = link_to_robot
        self.do_ranging = do_ranging
        self.tf_prefix = tf_prefix
        
        self.distance_1 = pzx.DeviceRange()
        self.distance_2 = pzx.DeviceRange()
        self.distance_3 = pzx.DeviceRange()
        self.distance_4 = pzx.DeviceRange()
        
        self.A = robot_list[robot_number]['left']
        self.B = robot_list[robot_number]['right']
        
        if robot_number == 1:
            self.C = robot_list[2]['left']
            self.D = robot_list[2]['right']
            self.destination = robot_list[2]['left']
        elif robot_number == 2:
            self.C = robot_list[1]['left']
            self.D = robot_list[1]['right']
            self.destination = robot_list[1]['left']
        
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
        self.f1.R = R
        self.f1.alpha = alpha
        
        self.f2 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f2.P = 1
        self.f2.H = np.array([[1.]])
        self.f2.F = np.array([[1.]])
        self.f2.B = np.array([[1.]])
        self.f2.Q = noise
        self.f2.R = R
        self.f2.alpha = alpha
        
        self.f3 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f3.P = 1
        self.f3.H = np.array([[1.]])
        self.f3.F = np.array([[1.]])
        self.f3.B = np.array([[1.]])
        self.f3.Q = noise
        self.f3.R = R
        self.f3.alpha = alpha
        
        self.f4 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f4.P = 1
        self.f4.H = np.array([[1.]])
        self.f4.F = np.array([[1.]])
        self.f4.B = np.array([[1.]])
        self.f4.Q = noise
        self.f4.R = R
        self.f4.alpha = alpha
        
        self.f5 = KalmanFilter(dim_x=1, dim_z=1, dim_u=1)
        self.f5.P = 1
        self.f5.H = np.array([[1.]])
        self.f5.F = np.array([[1.]])
        self.f5.B = np.array([[1.]])
        self.f5.Q = 0.001
        self.f5.R = 0.3
        self.f5.alpha = 1
        
        self.pozyx.setRangingProtocol(self.ranging_protocol)
        self.br = tf.TransformBroadcaster()
    
    def getDistance(self):
        try:
            return trf.checkDistance()
        except Exception as e:
            rospy.logwarn(e)
            pass
        
              
    def getDistances(self):
        # Distance 1 = AC
        # Distance 2 = AD
        # Distance 3 = BC
        # Distance 4 = BD
        # Where A(left) B(right) are local and C(left) D(right) are remote
        
        self.f1.predict()
        self.f2.predict()
        self.f3.predict()
        self.f4.predict()
        
        if self.do_ranging == 0:
            self.pozyx.rangingWithoutCheck(self.C, self.distance_1)
            self.pozyx.rangingWithoutCheck(self.D, self.distance_3)
            self.pozyx.rangingWithoutCheck(self.C, self.distance_2, self.B)        
            self.pozyx.rangingWithoutCheck(self.D, self.distance_4, self.B)   
        elif self.do_ranging == 1:
            self.pozyx.doRanging(self.C, self.distance_1)
            self.pozyx.doRanging(self.D, self.distance_3)
            self.pozyx.doRanging(self.C, self.distance_2, self.B)        
            self.pozyx.doRanging(self.D, self.distance_4, self.B)
        
        if self.distance_1[1] == 0 or self.distance_1[1] > 1000000:
            self.distance_1[1] = self.distance_prev_1
        if self.distance_2[1] == 0 or self.distance_2[1] > 1000000:
            self.distance_2[1] = self.distance_prev_2
        if self.distance_3[1] == 0 or self.distance_3[1] > 1000000:
            self.distance_3[1] = self.distance_prev_3
        if self.distance_4[1] == 0 or self.distance_4[1] > 1000000:
            self.distance_4[1] = self.distance_prev_4
        
        self.distance_prev_1 = self.distance_1[1]
        self.distance_prev_2 = self.distance_2[1]
        self.distance_prev_3 = self.distance_3[1]
        self.distance_prev_4 = self.distance_4[1]
                
        self.f1.update(self.distance_1[1])
        self.f2.update(self.distance_2[1])
        self.f3.update(self.distance_3[1])
        self.f4.update(self.distance_4[1])  

        return self.f1.x[0] * 0.001
        
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
        
#        self.f5.update(ROBOT_w_1)
#        ROBOT_w_1 = self.f5.x[0]
        
        ROBOT_w_2 = math.tan((LEFT_x_2 - RIGHT_x_2) / (LEFT_y_2 - RIGHT_y_2))
        if LEFT_y_2 < RIGHT_y_2:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        else:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        ROBOT_w_2 = math.radians(ROBOT_w_2) * -1
        
        self.br.sendTransform((LEFT_x_1, LEFT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     self.tf_prefix + "/left_tag_1",
                     self.link_to_robot)
        self.br.sendTransform((LEFT_x_2, LEFT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     self.tf_prefix + "/left_tag_2",
                     self.link_to_robot)
        self.br.sendTransform((RIGHT_x_1, RIGHT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     self.tf_prefix + "/right_tag_1",
                     self.link_to_robot)
        self.br.sendTransform((RIGHT_x_2, RIGHT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     self.tf_prefix + "/right_tag_2",
                     self.link_to_robot)
        self.br.sendTransform((ROBOT_x_1, ROBOT_y_1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, ROBOT_w_1),
                     rospy.Time.now(),
                     self.tf_prefix + "/robot_pos_1",
                     self.link_to_robot)
        self.br.sendTransform((ROBOT_x_2, ROBOT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, ROBOT_w_2),
                     rospy.Time.now(),
                     self.tf_prefix + "/robot_pos_2",
                     self.link_to_robot)

class Communicate(object):
    def __init__(self, pozyx):
        if robot_number == 1:
            self.destination = robot_list[2]['left']
        elif robot_number == 2:
            self.destination = robot_list[1]['left']
        self.pozyx = pozyx
        self.read_data = ""
        self.odom_data = Odometry()
        self.odom_data_rx = Odometry()
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
        data = Data([ord(c) for c in comp_data])
        self.pozyx.sendData(self.destination, data)
                
    def rxData(self):       
        self.pozyx.getRxInfo(self.rx_info)
        data = Data([0]*self.rx_info[1])
        self.pozyx.readRXBufferData(data)   
        message = str(self.rx_info[1]) 
        for i in data:
            message = message + chr(i)
        print(len(message))
        s = zlib.decompress(message)
        y = json.loads(s)

        odom_data_pub = Odometry()
        
        odom_data_pub.pose.pose.position.x = y['a']
        odom_data_pub.pose.pose.position.y = y['b']
        odom_data_pub.pose.pose.orientation.z = y['c']
        odom_data_pub.pose.pose.orientation.w = y['d']
        odom_data_pub.twist.twist.linear.x = y['e']
        odom_data_pub.twist.twist.angular.z = y['f']
        
        self.odom_data_rx = odom_data_pub
        rospy.set_param('~odom_rx', 1)
        
    def returnRxOdom(self):
        return self.odom_data_rx
        
def main():
    if rospy.get_param('~_ranging') == 1:
        distance = loc.getDistances()
    elif rospy.get_param('~_ranging') == 0:
        distance = loc.getDistance()
        
    rospy.loginfo(distance)
    rospy.loginfo("Range state: %i, Com state: %i", rospy.get_param('~_ranging'), rospy.get_param('~odom_rx'))
    
    if distance < loc_dis and distance > com_dis:
        rospy.set_param('~_ranging', 1)
        rospy.set_param('~zero_state', 1)
        
        loc.triangulationLocal()
        
        try:
            trf.getTransformData()
        except Exception as e:
            rospy.logwarn(e)
            pass
        
    elif distance <= com_dis:
        rospy.set_param('~_ranging', 0)
        
        try:
            com.txData()
            com.rxData()
            trf.odomData()
        except Exception as e:
            rospy.logwarn(e)
            pass
        
        if rospy.get_param('~zero_state') == 1 and rospy.get_param('~odom_rx') == 1:
            trf.calcZero()
            rospy.set_param('~zero_state', 0)
            
        elif rospy.get_param('~zero_state') == 0 and rospy.get_param('~odom_rx') == 1:
            trf.publishCF()
            trf.publishOF()
        else:
            pass
        
        pub.publish(com.returnRxOdom())
        
    elif distance < loc_dis and distance > com_dis and rospy.get_param('~do_ranging') == 0:
        rospy.set_param('~do_ranging', 1)
        
    elif distance >= loc_dis + 1:
        pass
    
if __name__ == "__main__":
    rospy.init_node('uwb_node')
    rospy.loginfo("Intializing UWB")
    serial_port = str(rospy.get_param('~serial_port', pzx.get_first_pozyx_serial_port()))
    frequency = float(rospy.get_param('~frequency', 10))
    rate = rospy.Rate(frequency)
    
    dt = 1000/frequency
    
    alpha = float(rospy.get_param('~alpha', 0.1))
    noise = float(rospy.get_param('~noise', 1))
    R = float(rospy.get_param('~R', 30))
    
    robot_number = rospy.get_param('~robot_number')
    
    left_tag_pos_x = float(rospy.get_param('~left_tag_pos_x'))
    left_tag_pos_y = float(rospy.get_param('~left_tag_pos_y'))
    right_tag_pos_x = float(rospy.get_param('~right_tag_pos_x'))
    right_tag_pos_y = float(rospy.get_param('~right_tag_pos_y'))
    link_to_robot = str(rospy.get_param('~link', 'base_footprint'))
    do_ranging = rospy.get_param('~do_ranging', 1)
    
    tf_prefix = str(rospy.get_param('~tf_prefix', 'base_footprint'))
    
    loc_dis = float(rospy.get_param('~loc_dis', 6))
    com_dis = float(rospy.get_param('~com_dis', 4))
    
    tag_pos = [left_tag_pos_x, left_tag_pos_y, right_tag_pos_x, right_tag_pos_y]
    
    protocol = str(rospy.get_param('~protocol', '0')) 
    
    if protocol == '0':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_FAST
    elif protocol == '1':
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_PRECISION
    else:
        rospy.logerr("Wrong value given for protocol. Either give: 'fast' or 'precise'")
        ranging_protocol = pzx.POZYX_RANGE_PROTOCOL_PRECISION
        
    pozyx = pzx.PozyxSerial(serial_port)
    pozyx.setRangingProtocol(ranging_protocol)
    stream = open(os.path.dirname(os.path.realpath(__file__)) + "/robot_list.yaml", "r")
    robot_list = yaml.load(stream)
    
    tx_topic = str(rospy.get_param('~tx_topic', 'uwb_server_tx'))
    rx_topic = str(rospy.get_param('~rx_topic', 'uwb_server_rx'))
    
    pub = rospy.Publisher(rx_topic, Odometry, queue_size = 10)
    
    loc = Localize(pozyx, dt, ranging_protocol, robot_list, tag_pos, robot_number, alpha, noise, R, link_to_robot, do_ranging, tf_prefix)
    com = Communicate(pozyx)
    trf = Transform(link_to_robot, tf_prefix)
    
    rospy.Subscriber(tx_topic, Odometry, com.odomData)
    
    rospy.set_param('~_ranging', 1)
    rospy.set_param('~odom_rx', 0)
    
    for i in range(25):
        distance = loc.getDistance()
        rate.sleep()
    rospy.loginfo("Done intializing UWB")
    
    while not rospy.is_shutdown():
        main()        
        rate.sleep()