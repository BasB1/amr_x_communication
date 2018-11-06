#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

rospy.init_node('twist')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size = 10)
msg = Twist()

freq = 100
speed = 0.2 #m/s
radius = 1

msg.linear.x = speed

distance = float(2 * math.pi * radius)
time = float(distance / speed)
angular = (2 * math.pi) / time
rate = rospy.Rate(freq)
count = 0
stop_count = int(time * freq)

msg.angular.z = angular

while not rospy.is_shutdown():
    if count < stop_count:
        pub.publish(msg)
    elif count > stop_count:
        break
    count += 1 
    rate.sleep()
rospy.loginfo("Done with twisting")