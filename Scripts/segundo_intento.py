#!/usr/bin/env python3
from ast import Pass
from typing_extensions import Self
from unittest import skip
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
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
        self.pub3 = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=1)

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
    
    def do_rotation(self):
        
        self.pub3.publish(Empty())
        grados = math.degrees(self.yaw)

        g_g = 90
        
        if grados < 0:
            grados = grados + 360
        
        rotacion_terminada = False
        while rotacion_terminada == False:
            grados = math.degrees(self.yaw)
            if g_g-grados > 0:
                print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = 1
                self.pub.publish(self.cmd)    
            if g_g-grados <= 0:    
                rotacion_terminada = True
                self.cmd.angular.z = 0            
                self.pub.publish(self.cmd)
                
        self.pub3.publish(Empty())

    def do_negative_rotation(self):

        print("hola")        
        self.pub3.publish(Empty())
        grados = math.degrees(self.yaw)
        print(grados)
        g_g = -90        
        
        rotacion_terminada = False
        while rotacion_terminada == False:
            grados = math.degrees(self.yaw)
            if abs(g_g)-abs(grados) > 0:
                # print("EL GRADO INICIAL ES ", g_g)
                print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = -1
                # print("avanza")
                self.pub.publish(self.cmd)    
                # self.r.sleep()
            if abs(g_g)-abs(grados) <= 0:    
                rotacion_terminada = True
                self.cmd.angular.z = 0            
                self.pub.publish(self.cmd)        

    def movimiento(self):
        state = 0

        if state == 0:    
            self.do_rotation()
            state = 1
        
        if state == 1:
            c=0
            while c<20000000:    
                c= c+1
            state = 2    
        
        
        if state == 2:
            c=0
            while c < 100000:
                self.cmd.linear.x = 0.5
                self.pub.publish(self.cmd)
                c=c+1
            self.cmd.linear.x = 0
            self.pub.publish(self.cmd)    
            state = 3

        if state == 3:
            c=0
            while c<20000000:    
                c= c+1
            state = 4  


        if state == 4:
            self.do_negative_rotation() 
            
    def start(self):
        c=0
        while c<20000000:    
            c= c+1
            
        if self.first_odom_check:  
            c = 0
            
        self.movimiento()   
            
        print("BUCLE TERMINADO")


if __name__ == '__main__':

    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
