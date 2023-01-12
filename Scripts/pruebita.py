#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

rospy.init_node('my_quaternion_to_euler')
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)


def get_rotation(msg):
    mover = Twist()
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    #print(msg.pose.pose.orientation)#Imprime la orientación del robot en tiempo real
    print(math.degrees(yaw))

    grados = math.degrees(yaw)

    if grados > 17 and grados < 110:
        mover.angular.z = 1
    elif grados > 110:
        mover.angular.z = 0

    pub.publish(mover)

sub = rospy.Subscriber('/odom', Odometry, get_rotation)


r = rospy.Rate(1)
while not rospy.is_shutdown():
    r.sleep()