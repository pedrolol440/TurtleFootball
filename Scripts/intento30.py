#!/usr/bin/env python3
from ast import Pass
from logging import info
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
        self.estado=0
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
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
 

    def callback2(self, msg):
        # rospy.loginfo('Image received...')
        self.image_rgb = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

    def get_rotation(self,msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        
      



    
    def do_rotation(self):
        
        self.pub3.publish(Empty())
        grados = math.degrees(self.yaw)

        # print("LOS DE LA FUNCION ",grados)
        g_g = 90
        # g_g = math.degrees(self.grados_goal)
        
        if grados < 0:
            grados = grados + 360
        
        rotacion_terminada = False
        while rotacion_terminada == False:

            grados = math.degrees(self.yaw)
            if g_g-grados > 0:
                # print("EL GRADO INICIAL ES ", g_g)
                print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = .7
                # print("avanza")
                self.pub.publish(self.cmd)    
                # self.r.sleep()
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
        # print("LOS DE LA FUNCION ",grados)
        g_g = -90
        # g_g = math.degrees(self.grados_goal)
        
        
        
        rotacion_terminada = False
        while rotacion_terminada == False:

            grados = math.degrees(self.yaw)
            if abs(g_g)-abs(grados) > 0:
                # print("EL GRADO INICIAL ES ", g_g)
                print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = -.7
                # print("avanza")
                self.pub.publish(self.cmd)    
                # self.r.sleep()
            if abs(g_g)-abs(grados) <= 0:    
        
                rotacion_terminada = True
                self.cmd.angular.z = 0            
                self.pub.publish(self.cmd)       
        self.pub3.publish(Empty())         

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
                self.cmd.linear.x = 0.2
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
            self.pub3.publish(Empty())         
            self.do_negative_rotation()
        
        if state == 5:
            c=0
            while c<20000000:    
                c= c+1            

    def movimiento2(self):
        state = 0
        self.pub3.publish(Empty())         
        if state == 0:    
            self.do_negative_rotation()
            state = 1
        
        if state == 1:
            c=0
            while c<20000000:    
                c= c+1
            state = 2    
        
        
        if state == 2:
            c=0
            while c < 100000:
                self.cmd.linear.x = 0.2
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
            self.do_rotation()
            state = 5
        
        if state == 5:
            c=0
            while c<20000000:    
                c= c+1
              
    



    def start(self):
            
        
        # self.pub3.publish(Empty())
        c=0
        while c<20000000:    
            c= c+1
            
        if self.first_odom_check:  
            c = 0
            
       
        # self.movimiento2()    
        
        # self.movimiento2()
        # self.movimiento2()   
            
        while True:
            #if self.image is not None and self.vagabundeo.data==False:
            # frame = "a"
            if self.estado==5:
                c=0
                while c < 1000000:
                    self.cmd.linear.x = 1
                    self.pub.publish(self.cmd)
                    c=c+1
                self.cmd.linear.x = 0
                self.pub.publish(self.cmd) 
                break
            if (self.image_rgb is not None) and (self.image is not None):
                frame = self.image_rgb
                # if frame=="a":
                #     skip
                hight,width,_ = frame.shape
                # Conversión de espacio RGB a HSV
            
                HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                x,y,w,h=0,0,0,0
                # # 2- define the range of red
                # lower=np.array([100, 0, 0])
                # upper=np.array([255, 255,255])

                # #check if the HSV of the frame is lower or upper red
                # Red_mask = cv2.inRange(HSV,lower, upper)
                # result = cv2.bitwise_and(frame, frame, mask = Red_mask)
                    #Rango y máscara para el color rojo

                red_lower = np.array([136, 87, 111], np.uint8)
                red_upper = np.array([180, 255, 255], np.uint8)
                # red_lower = np.array([250, 82, 84], np.uint8)
                # red_upper = np.array([7, 102, 104], np.uint8)                
                red_mask = cv2.inRange(HSV, red_lower, red_upper)
                print(red_mask)

                                    #Rango y máscara para el color verde
                green_lower = np.array([20, 100, 100], np.uint8)
                green_upper = np.array([30, 255, 255], np.uint8)
                green_mask = cv2.inRange(HSV, green_lower, green_upper)


                #Definimos kernel para la operación morfológica
                kernal = np.ones((5, 5), "uint8")

                # For red color
                red_mask = cv2.dilate(red_mask, kernal)
                res_red = cv2.bitwise_and(frame, frame,
                                        mask=red_mask)

                # For green color
                green_mask = cv2.dilate(green_mask, kernal)
                res_green = cv2.bitwise_and(frame, frame,
                                            mask=green_mask)                    

                # Draw rectangular bounded line on the detected red area
                contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # Draw rectangular bounded line on the detected red area
                contours_g, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                if len(contours) != 0 and len(contours_g):

                                    # Find the index of the largest contour
                    areas = [cv2.contourArea(c) for c in contours]
                    if areas != 0:
                        max_index = np.argmax(areas)
                        red_cnt=contours[max_index]
                        x_r,y_r,w_r,h_r = cv2.boundingRect(red_cnt)
                        frame = cv2.rectangle(frame, (x_r, y_r), (x_r+w_r, y_r+h_r), (0, 0, 255), 2)
                        cX = int((x_r+x_r+w_r)/2)
                        if cX >480:
                            cX = 479
                        cY = int((y_r+y_r+h_r)/2)
                        if cY >480:
                            cY = 479                          
                        cv2.circle(frame, (cX,cY), 5, (255,255,255), -1)

                    areas_g = [cv2.contourArea(d) for d in contours_g]
                    if areas_g != 0:
                        max_index_g = np.argmax(areas_g)
                        green_cnt=contours_g[max_index_g]
                        x_g,y_g,w_g,h_g = cv2.boundingRect(green_cnt)
                        frame = cv2.rectangle(frame, (x_g, y_g), (x_g+w_g, y_g+h_g), (0, 255, 0), 2)

                        # M = cv2.moments(red_cnt)
                        # cX= int(M["m10"] / M["m00"])
                        # cY= int(M["m01"] / M["m00"])
                        

                        
                        cX_g = int((x_g+x_g+w_g)/2)
                        if cX_g >480:
                            cX_g = 479                            
                        cY_g = int((y_g+y_g+h_g)/2)
                        if cY_g >480:
                            cY_g = 479                           
                        cv2.circle(frame, (cX_g,cY_g), 5, (0,255,255), -1)

                    cv2.circle(frame, (0,0), 5, (255,255,255), -1)

                    

                    
                    

                    # TODO: Anadir la funcion que lea la profundidad de los objetos en camara
                    if areas_g != 0 and areas != 0:
                        distancia_media = self.image[cX][cY]
                        distancia_verde = self.image[cX_g][cY_g]
                        print("distancia rojo: " + str(distancia_media))
                        print("distancia amarillo: " + str(distancia_verde))    
                                

                        # TODO: Movernos hasta un punto cercano (a decidir) a la pelota 

                        # if distancia_verde > 30:
                        #     self.cmd.linear.x = 0.4
                        # if distancia_verde <=30:
                        #     self.cmd.linear.x = 0
                        dx = cX_g - cX
                        threshold = 10
                        
                        #     if dx > 0:                                 
                        if abs(dx) > threshold:

                            # Crear el mensaje de velocidad
                            #movimiento(dx)
                                
                            # Establecer la velocidad angular para girar el TurtleBot hacia la derecha o la izquierda
                            #si la bola esta a la dcha(girar a la dcha)
                            if dx > 0:
                                self.movimiento2()
                            else:#si esta a la izq
                                self.movimiento()
                        if abs(dx) < threshold:
                            self.estado = 1 
                            print("duro de cojones")
                            
                                                    
             
                        if self.estado == 1:
                            print(cX_g - 240)   
                            if abs(cX_g - 240)  >= threshold:
                                print("celebremos por el dinero que hacemos")
                                self.cmd.angular.z = -0.2
                                self.pub.publish(self.cmd)

                            if abs(cX_g - 240)  < threshold:
                                print("soy como pablo en medellin")
                                self.cmd.angular.z = 0
                                self.pub.publish(self.cmd)
                                self.estado=5
                            
                        
                   
                               
                                
                        #     detectar_centroides()
                        #     dx = centroidebola.x - centroiderojo.x

                    
                print("Me llaman de Munich")
                # self.cmd.linear.x = 0.5
                # self.pub.publish(self.cmd)

                # Program Termination
                
                # self.pub.publish(cmd)
                cv2.imshow("Que miras bobo", frame)
                if cv2.waitKey(10) & 0xFF == ord('q'):
                    cap.release()
                    cv2.destroyAllWindows()
                    break                                                      

    


if __name__ == '__main__':

    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
    # while not rospy.is_shutdown():
        # my_node.r.sleep()
    #my_node.pase_al_rojo()
