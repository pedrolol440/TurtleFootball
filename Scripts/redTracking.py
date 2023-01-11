#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import os
import numpy as np


class Nodo(object):
    def __init__(self):
        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub_vag = rospy.Publisher('/vagabundeo', Bool, queue_size=10)
        self.vagabundeo = Bool()
        self.vagabundeo.data = False
        # Subscribers
        self.sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.callback)
        self.pub = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=5)

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.br.imgmsg_to_cv2(msg)


    def start(self):
            while True:
                if self.image is not None and self.vagabundeo.data==False:
                    frame = self.image
                    hight,width,_ = frame.shape
                    HSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                    x,y,w,h=0,0,0,0
                    # 2- define the range of red
                    lower=np.array([100, 0, 0])
                    upper=np.array([255, 255,255])

                    #check if the HSV of the frame is lower or upper red
                    Red_mask = cv2.inRange(HSV,lower, upper)
                    result = cv2.bitwise_and(frame, frame, mask = Red_mask)

                    # Draw rectangular bounded line on the detected red area
                    contours, hierarchy = cv2.findContours(Red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    for pic,contour in enumerate(contours):
                        area = cv2.contourArea(contour)
                        if(area > 1000): #to remove the noise
                            # Constructing the size of boxes to be drawn around the detected red area
                            x,y,w,h = cv2.boundingRect(contour)
                            frame = cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

                            cv2.imshow("Tracking Red Color",frame)
                            cv2.imshow("Mask",Red_mask)
                            cv2.imshow("And",result)

                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                            # print ("x", x+w)
                            # print ("y", y+h)
                            # print("P_hight", hight)
                            # print("P_width", width/2)
                            # print("#########################")
                            giro=(width/2-x-w)/width
                            avance=(hight-y-h)/hight
                            # Enviar comandos al robot
                            print("Avance:",avance)
                            print("Giro:", giro)
                            cmd = Twist()
                            cmd.linear.x = avance*0.5
                            cmd.angular.z = giro*0.5
                            self.pub.publish(cmd)

                            if abs(avance) <= 10**-1 and abs(giro) <= 10**-1:
                                print("Vagabundeo")
                                self.vagabundeo.data=True
                                self.pub_vag.publish(self.vagabundeo)
                                cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("tracking", anonymous=True)
    my_node = Nodo()
    my_node.start()
