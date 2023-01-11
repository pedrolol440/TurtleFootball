#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(data):
    valor = data.ranges[480]
    rospy.loginfo("distancia: %s",valor)

def nodo():
    mensaje = Twist()
    mensaje.angular.z = 0.5
    rospy.init_node("recibo_laser")
    rospy.Subscriber("/scan",LaserScan,callback)
    #rospy.spin()

    pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(mensaje)
        rate.sleep()

if __name__=='__main__':
    nodo()
