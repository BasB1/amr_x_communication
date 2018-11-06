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

class Transform():
    def __init__(self):
         self.broadcaster = tf.TransformBroadcaster()
         self.listener = tf.TransformListener()
         
    def getTransformData(self):
        (self.trans, self.rot) = self.listener.lookupTransform('/robot_pos_1', '/world', rospy.Time(0))
         
    def calcZero(self):
        try:
            self._x = self.trans[0] + (self.odom_data.pose.pose.position.x)
            self._y = self.trans[1] + (self.odom_data.pose.pose.position.y)

            quaternion = (
                self.odom_data.pose.pose.orientation.x,
                self.odom_data.pose.pose.orientation.y,
                self.odom_data.pose.pose.orientation.z,
                self.odom_data.pose.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            
            self._z = euler[2] + self.rot[2]  + math.pi
            rospy.sleep(0.1 / self.itter)
        except:
            pass
        
    def publishCF(self):
        self.broadcaster.sendTransform((self._x, self._y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, self._z),
                     rospy.Time.now(),
                     "child_frame",
                     "world")
    
    def publishOF(self, x, y, z):
        self.broadcaster.sendTransform((x, y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, z),
                     rospy.Time.now(),
                     "base_footprint",
                     "child_frame")

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
        self.f5.Q = 0.001
        self.f5.R = 0.1
        self.f5.alpha = 1
        
        self.pozyx.setRangingProtocol(self.ranging_protocol)
        self.br = tf.TransformBroadcaster()

    def doRanging(self):        
        self.f1.predict()
        self.pozyx.doRanging(self.C, self.distance_1)
        self.f1.update(self.distance_1[1])
        return self.f1.x[0] * 0.001
              
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
                
        self.f1.update(self.distance_1[1])
        self.f2.update(self.distance_2[1])
        self.f3.update(self.distance_3[1])
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
        
        self.f5.update(ROBOT_w_1)
        
        ROBOT_w_2 = math.tan((LEFT_x_2 - RIGHT_x_2) / (LEFT_y_2 - RIGHT_y_2))
        if LEFT_y_2 < RIGHT_y_2:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        else:
            ROBOT_w_2 = math.degrees(ROBOT_w_2)
        ROBOT_w_2 = math.radians(ROBOT_w_2) * -1
        
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
                     tf.transformations.quaternion_from_euler(0, 0, ROBOT_w_1),
                     rospy.Time.now(),
                     "robot_pos_1",
                     "world")
        self.br.sendTransform((ROBOT_x_2, ROBOT_y_2, 0),
                     tf.transformations.quaternion_from_euler(0, 0, ROBOT_w_2),
                     rospy.Time.now(),
                     "robot_pos_2",
                     "world")

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
           "p":  #pose
                {"px": round(self.odom_data.pose.pose.position.x, 3), #position
                "py": round(self.odom_data.pose.pose.position.y, 3),
                "pz": round(self.odom_data.pose.pose.position.z, 3),
                "ox": round(self.odom_data.pose.pose.orientation.x, 3), #orientation
                "oy": round(self.odom_data.pose.pose.orientation.y, 3),
                "oz": round(self.odom_data.pose.pose.orientation.z, 3),
                "ow": round(self.odom_data.pose.pose.orientation.w, 3)},
            "t":  #twist
                {"lx": round(self.odom_data.twist.twist.linear.x, 3), #linear
                "ly": round(self.odom_data.twist.twist.linear.y, 3),
                "lz": round(self.odom_data.twist.twist.linear.z, 3),
                "ax": round(self.odom_data.twist.twist.angular.x, 3), #angulaer
                "ay": round(self.odom_data.twist.twist.angular.y, 3),
                "az": round(self.odom_data.twist.twist.angular.z, 3)}
            }
        s = json.dumps(x)
        comp_data = zlib.compress(str(s))
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
        
        odom_data_pub.pose.pose.position.x = y['p']['px']
        odom_data_pub.pose.pose.position.y = y['p']['py']
        odom_data_pub.pose.pose.position.z = y['p']['pz']
        odom_data_pub.pose.pose.orientation.x = y['p']['ox']
        odom_data_pub.pose.pose.orientation.y = y['p']['oy']
        odom_data_pub.pose.pose.orientation.z = y['p']['oz']
        odom_data_pub.pose.pose.orientation.w = y['p']['ow']

        odom_data_pub.twist.twist.linear.x = y['t']['lx']
        odom_data_pub.twist.twist.linear.y = y['t']['ly']
        odom_data_pub.twist.twist.linear.z = y['t']['lz']
        odom_data_pub.twist.twist.angular.x = y['t']['ax']
        odom_data_pub.twist.twist.angular.y = y['t']['ay']
        odom_data_pub.twist.twist.angular.z = y['t']['az']
        
        return odom_data_pub

def main():
    distance = loc.doRanging()
    rospy.loginfo(distance)
    if distance < loc_dis:
        stat = 'loc'
    elif distance <= com_dis:
        stat = 'com'
    elif distance >= loc_dis + 1:
        stat = 'none'
    
    if stat == 'loc':
        loc.getDistances()
        loc.triangulationLocal()
        try:
            trf.getTransformData()
        except Exception as e:
            pass
    elif stat == 'com':
        trf.calcZero()
        try:
            odom_data = com.rxData()
            pub.publish(odom_data)
        except Exception as e:
            odom_data = Odometry()
            pass
        com.txData()
        x = odom_data.pose.pose.position.x
        y = odom_data.pose.pose.position.y
        z = odom_data.pose.pose.position.z
        trf.publishOF(x, y, z)
    elif stat == 'none':
        pass
        
    
    

if __name__ == "__main__":
    rospy.init_node('uwb_node')
    
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
    
    loc_dis = float(rospy.get_param('~loc_dis', 6))
    com_dis = float(rospy.get_param('~com_dis', 4))
    
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
    
    destination = rospy.get_param('~destination', 0x6e2f)
    tx_topic = str(rospy.get_param('~tx_topic', 'uwb_server_tx'))
    rx_topic = str(rospy.get_param('~rx_topic', 'uwb_server_rx'))
    
    pub = rospy.Publisher(rx_topic, Odometry, queue_size = 10)
    
    loc = Localize(pozyx, dt, ranging_protocol, robot_list, tag_pos, robot_number, alpha, noise)
    com = Communicate(pozyx, destination)
    trf = Transform()
    
    rospy.Subscriber(tx_topic, Odometry, com.odomData)
    
    while not rospy.is_shutdown():
        main()         
        rate.sleep()