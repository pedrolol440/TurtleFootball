#!/usr/bin/env python3
from ast import Pass
from typing_extensions import Self
from unittest import skip
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point, Twist
from math import atan2
import math


class Nodo(object):
    def __init__(self):
        # Params
        self.r = rospy.Rate(1)
        self.image = None
        self.br = CvBridge()
        self.cmd = Twist()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.first_odom = 4
        self.grados_goal =  0.0
        self.first_odom_check = False
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.image_rgb = None
        # Publishers
       
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
        self.pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        

        # Subscribers
        self.sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.callback)
        self.sub2 = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback2)
        
        self.sub3 = rospy.Subscriber('/odom', Odometry, self.get_rotation)
        

    

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def callback2(self, msg):
        self.image_rgb = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def get_rotation(self,msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        if self.first_odom > 0:
            self.first_odom = self.first_odom-1
            print(self.first_odom)
        if self.first_odom == 0 and self.first_odom_check == False:
            self.grados_goal = self.yaw 
            self.first_odom_check = True
    
    def do_rotation(self):
        
        grados = math.degrees(self.yaw)
        print("LOS DE LA FUNCION ",grados)
        g_g = 0.0
        g_g = math.degrees(self.grados_goal)
        if g_g < 0:
            g_g = g_g + 360
        if grados < 0:
            grados = grados + 360
        print("EL GRADO INICIAL ES ", g_g)
        print("ACTUALMENTE EN",grados)
        if grados >= g_g and grados < g_g + 90:
            print("EL GRADO INICIAL ES ", g_g)
            print("ACTUALMENTE EN",grados)        
            self.cmd.angular.z = 1
            print("avanza")
            self.pub.publish(self.cmd)    
        if grados >= g_g + 90:
            self.cmd.angular.z = 0
            print("para")
            self.pub.publish(self.cmd)

    def do_negative_rotation(self):
        grados = math.degrees(self.yaw)
        g_g = 0.0
        g_g = math.degrees(self.grados_goal)
        if g_g < 0:
            g_g = g_g + 360
        if grados <= 0:
            grados = grados + 360

        tresjol = grados - g_g     
        print("LOS DE LA FUNCION ",grados)
        print("EL GRADO INICIAL ES ", g_g)
        print("EL TRESJOL DEL OS COJONES ES", tresjol)
        if abs(tresjol) > 1.0:
            print("EL GRADO INICIAL ES ", g_g)
            print("ACTUALMENTE EN",grados)        
            self.cmd.angular.z = -1
            print("avanza")
            self.pub.publish(self.cmd)    
        if abs(tresjol) <=1.0:
            self.cmd.angular.z = 0
            print("para")
            self.pub.publish(self.cmd)

    def start(self):
        print(math.degrees(self.yaw))
        c = 0
        while c<20000000:    
            c= c+1
            if self.first_odom_check:    
                self.do_rotation()
            
        print("BUCLE TERMINADO")        

if __name__ == '__main__':

    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
