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

        # Variable de la clase
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
        self.cX_g=0
        
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)
        self.image_rgb = None
        
        # Publishers
       
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5) #Publica movimientos en el robot real
        self.pub2 = rospy.Publisher("/cmd_vel", Twist, queue_size=5)  #Publica movimientos en el robot simulado
        self.pub3 = rospy.Publisher("/mobile_base/commands/reset_odometry", Empty, queue_size=1) #Resetea la odometría

        # Subscribers
        self.sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.callback) #Para obetener el frame de la camara de profundidad
        self.sub2 = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback2) #Para obetener el frame de la camara de color
        
        self.sub3 = rospy.Subscriber('/odom', Odometry, self.get_rotation) #Para obtener la odometría del robot
        

    
    #Función callback que obtiene la imagen de profundidad
    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)
 
    #Función callback que obtiene la imagen de color
    def callback2(self, msg):
        # rospy.loginfo('Image received...')
        self.image_rgb = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    #Función callback que obtiene las orientaciones del robot de la odometria y las pasa a grados euler
    def get_rotation(self,msg):

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        
      


    #Función que hace rotar al robot +90 grados
    def do_rotation(self):
        
        #Reseteamos la odometria
        self.pub3.publish(Empty())

        #Conversión a grados sexagesimales
        grados = math.degrees(self.yaw)
        
        #Se define los grados que se quiere girar
        g_g = 90
        
        #Conversión en caso de obtener grados negativos de la odometría
        if grados < 0:
            grados = grados + 360
        

        rotacion_terminada = False #condición de terminación del bucle de giro

        #BUCLE DE GIRO: se actualiza el valor de los grados actuales en cada iteracion,
        # se restan los grados desesados a los actuales y se mantiene una velocidad de rotación
        # mientras la resta sea positiva. Al finalizar se detiene el robot. 
        while rotacion_terminada == False:

            grados = math.degrees(self.yaw)
            if g_g-grados > 0:
                # print("EL GRADO INICIAL ES ", g_g)
                #print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = .4
                # print("avanza")
                self.pub.publish(self.cmd)    
                # self.r.sleep()
            if g_g-grados <= 0:    
        
                rotacion_terminada = True
                self.cmd.angular.z = 0            
                self.pub.publish(self.cmd)
        self.pub3.publish(Empty()) #Volvemos a resetear la odometria


    #Función que hace rotar al robot -90 grados
    
    # El funcionamiento es el mismo que el movimiento de +90 grados, pero indicando como target -90 y 
    # en la resta relizando el valor absoluto para evitar problemas en la lógica.
    def do_negative_rotation(self):

        #print("hola")        
        self.pub3.publish(Empty())
        grados = math.degrees(self.yaw)
        #print(grados)
        # print("LOS DE LA FUNCION ",grados)
        g_g = -90
        # g_g = math.degrees(self.grados_goal)
        
        
        
        rotacion_terminada = False
        while rotacion_terminada == False:

            grados = math.degrees(self.yaw)
            if abs(g_g)-abs(grados) > 0:
                # print("EL GRADO INICIAL ES ", g_g)
                #print("ACTUALMENTE EN",grados)        
                self.cmd.angular.z = -.4
                # print("avanza")
                self.pub.publish(self.cmd)    
                # self.r.sleep()
            if abs(g_g)-abs(grados) <= 0:    
        
                rotacion_terminada = True
                self.cmd.angular.z = 0            
                self.pub.publish(self.cmd)       
        self.pub3.publish(Empty())         


    #Función donde se implementa el movimiento hacia la izquierda del robot:

    #-Se implementa una sencilla máquina de estados que llama primero a la rotación de +90º,
    # posteriormente aplica una velocidad lineal durante un tiempo limitado y finalmente realiza una rotación
    # de -90º para volver a la orientación inicial. Entre estados, se introducen pequeños bucles vacios que funcionan a modo
    # de ligeros retardos para evitar que los movimientos se superpongan.
    def movimiento(self):
        state = 0

        if state == 0:   
            print("giro 90 positivo") 
            self.do_rotation()
            state = 1
        
        if state == 1:
            c=0
            while c<20000000:    
                c= c+1
            state = 2    
        
        
        if state == 2:
            print("recto")
            c=0
            while c < 50000:
                self.cmd.linear.x = 0.1
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
            print("giro 90 negativo") 
            self.pub3.publish(Empty())         
            self.do_negative_rotation()
        
        if state == 5:
            c=0
            while c<20000000:    
                c= c+1            


    #Función que implementa el movimiento hacia la derecha del robot, identica a la anterior, pero con 
    # la diferencia de llamar primero a la rotación de -90º y posterioremente a la rotación de +90º.
    def movimiento2(self):
        state = 0
        self.pub3.publish(Empty())         
        if state == 0:    
            print("giro 90 positivo")
            self.do_negative_rotation()
            state = 1
        
        if state == 1:
            c=0
            while c<20000000:    
                c= c+1
            state = 2    
        
        
        if state == 2:
            print("recto")
            c=0
            while c < 50000:
                self.cmd.linear.x = 0.1
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
            print("giro 90 negativo")
            self.do_rotation()
            state = 5
        
        if state == 5:
            c=0
            while c<20000000:    
                c= c+1
              
    


    ##FUNCION PRINCIPAL ##
    #En esta se encuentra la implementación del programa.
    def start(self):
            
        
        # Ligero retardo que permite que se inicialicen los valores de la odometría
        c=0
        while c<20000000:    
            c= c+1            
        if self.first_odom_check:  
            c = 0

        #Bucle principal del programa                
        while True:
            
            
            ## Primero comprobamos que haya imagen y que nos encontramos en un estado de los disponibles ##
                         
            if (self.image_rgb is not None) and (self.image is not None) and self.estado<5 :
                frame = self.image_rgb
                
                
                
                hight,width,_ = frame.shape
                
                # Conversión de espacio RGB a HSV            
                HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                x,y,w,h=0,0,0,0                        
                
                #Rango y máscara para el color rojo
                red_lower = np.array([136, 87, 111], np.uint8)
                red_upper = np.array([180, 255, 255], np.uint8)                            
                red_mask = cv2.inRange(HSV, red_lower, red_upper)
                

                #Rango y máscara para el colora marillo (a pesar de poner green)
                green_lower = np.array([20, 100, 100], np.uint8)
                green_upper = np.array([30, 255, 255], np.uint8)
                green_mask = cv2.inRange(HSV, green_lower, green_upper)


                #Definimos kernel para la operación morfológica
                kernal = np.ones((5, 5), "uint8")

                #Operaciones morfológicas para definir mejor los contornos:

                    # Para el color rojo
                red_mask = cv2.dilate(red_mask, kernal)
                res_red = cv2.bitwise_and(frame, frame,
                                        mask=red_mask)

                    # Para el color amarillo
                green_mask = cv2.dilate(green_mask, kernal)
                res_green = cv2.bitwise_and(frame, frame,
                                            mask=green_mask)                    

                # Obtenemos los contornos detectados como rojos
                contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                # Obtenemos los contornos detectados como amarillos
                contours_g, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                #Si hemos encontrado contornos:
                if len(contours) != 0 and len(contours_g):

                    # Obtenemos las areas de los contornos rojos
                    areas = [cv2.contourArea(c) for c in contours]
                    
                    if areas != 0:

                        #Sacamos el area de mayor tamaño, que corresponde con el jugador
                        max_index = np.argmax(areas)
                        red_cnt=contours[max_index]

                        #Sacamos las coordenadas de su bounding box
                        x_r,y_r,w_r,h_r = cv2.boundingRect(red_cnt)

                        #Dibujamos la bounding box
                        frame = cv2.rectangle(frame, (x_r, y_r), (x_r+w_r, y_r+h_r), (0, 0, 255), 2)
                        
                        #Sacamos las coordenadas del centroide del jugador a partir de las coordenadas de la bounding box
                        # También debemos asegurarnos de no superar las dimensiones de la imagen
                        
                        #Coordenada X del centroide:
                        cX = int((x_r+x_r+w_r)/2)
                        if cX >480:
                            cX = 479
                        
                        #Coordenada Y del centroide:    
                        cY = int((y_r+y_r+h_r)/2)
                        if cY >480:
                            cY = 479

                        #Finalmente, se dibuja el centroide en la imagen 
                        cv2.circle(frame, (cX,cY), 5, (255,255,255), -1)

                    #Repetimos el proceso de deteccion del centroide de la pelota, analizando los contornos amarillos ahora:
                    areas_g = [cv2.contourArea(d) for d in contours_g]
                    if areas_g != 0:
                        max_index_g = np.argmax(areas_g)
                        green_cnt=contours_g[max_index_g]
                        x_g,y_g,w_g,h_g = cv2.boundingRect(green_cnt)
                        frame = cv2.rectangle(frame, (x_g, y_g), (x_g+w_g, y_g+h_g), (0, 255, 0), 2)

                       
                        self.cX_g = int((x_g+x_g+w_g)/2)
                        if self.cX_g >480:
                            self.cX_g = 479                            
                        cY_g = int((y_g+y_g+h_g)/2)
                        if cY_g >480:
                            cY_g = 479                           
                        cv2.circle(frame, (self.cX_g,cY_g), 5, (0,255,255), -1)

                    cv2.circle(frame, (0,0), 5, (255,255,255), -1)
                    cv2.imshow("Que miras bobo", frame)
                    

                    
                    

                    #Definición del estado actual (si nos encontramos en el estado 0,1 o 2)

                    #Si se han detectado los centroides
                    if areas_g != 0 and areas != 0:
                        
                        #Diferencia entre las coordenadas X de los centroides            
                        dx = self.cX_g - cX
                        print(dx)

                        #Umbral de alineación mínima 
                        threshold1 = 15
                        
                        #Si la diferencia entre los centroides es mayor que el umbral, es decir, no se ha alineado aun                             
                        if abs(dx) > threshold1:
                            
                            #Si la pelota esta a la derecha del jugador
                            if dx > 0 and self.estado !=3 and self.estado!=2:
                                self.estado=1

                            #Si la pelota esta a la izquierda del jugador
                            if dx < 0 and self.estado !=3 and self.estado!=1:
                                self.estado=2

                        #Si la diferencia entre los centroides es menor que el umbral, es decir, ya se han alineado
                        if abs(dx) < threshold1 :

                            #Si se encontraba la pelota a la derecha del jugador, se pasa al estado 3
                            if self.estado==1:
                                self.estado = 3 

                            #Si se encontraba la pelota a la izquierda del jugador, se pasa al estado 3
                            if self.estado==2:
                                self.estado = 4
                            
                            
            #Acciones a realizar según el estado

            #Estado 1 (la pelota está a la derecha del jugador y los centroides no estan alineados)
            if self.estado == 1:
                
                print("estado 1")

                #Se realiza un movimiento a la derecha para encontrar una nueva perspectiva
                self.movimiento2()

            #Estado 2 (la pelota está a la izquierda del jugador y los centroides no estan alineados)
            if self.estado ==2: #
                
                #Se realiza un movimiento a izquierda para encontrar una nueva perspectiva
                print("estado 2")
                self.movimiento()

            #Estado 3  el robot ya esta en la diagonal y tiene que orientarse hacia la pelota            
            if self.estado == 3:

                print("estado 3")

                # Se define otro umbral de alineación minima
                threshold2=10
                
                #Se obtiene difX y se compar con el umbral

                #Si el centroide de la pelota y el centro de la imagen no están alineados    
                if abs(self.cX_g - 240)  >= threshold2:
                    
                    #El robot comienza a girar sobre si mismo a la izquierda
                    self.cmd.angular.z = 0.3
                    self.pub.publish(self.cmd)

                #Si el centroide de la pelota y el centro de la imagen están alineados
                if abs(self.cX_g - 240)   < threshold2:
                    
                    #El robot para de girar y pasa al ultimo estado
                    self.cmd.angular.z = 0
                    self.pub.publish(self.cmd)
                    self.estado=5  

            #Estado 4  el robot ya esta en la diagonal y tiene que orientarse hacia la pelota 
            if self.estado == 4:

                print("estado 4")

                # Se define otro umbral de alineación minima
                threshold2=10
                
                #Se obtiene difX y se compar con el umbral

                 #Si el centroide de la pelota y el centro de la imagen no están alineados 
                if abs(self.cX_g - 240)  >= threshold2:
                    
                    #El robot comienza a girar sobre si mismo a la derecha
                    self.cmd.angular.z = -0.3
                    self.pub.publish(self.cmd)

                if abs(self.cX_g - 240)   < threshold2:
                    
                    #El robot para de girar y pasa al ultimo estado
                    self.cmd.angular.z = 0
                    self.pub.publish(self.cmd)
                    self.estado=5  

            #Estado 5 el robot ya esta correctamente posicionado y orientado para realizar el pase
            if self.estado==5:

                print("estado 5")

                #Avanza en linea recta durante un tiempo determinado por la variable c en el while(determinado experimentalmente)
                c=0
                while c < 60000:
                    self.cmd.linear.x = 1
                    self.pub.publish(self.cmd)
                    c=c+1
                #una vez se ha consumido este tiempo y se ha golpeado la pelota, se para el robot y acaba el programa
                self.cmd.linear.x = 0
                self.pub.publish(self.cmd) 
                break                   
                               
                                
                        #
            
                                                                    

    

#Función main donde se lanza el nodo y el programa principal de este
if __name__ == '__main__':

    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
    
