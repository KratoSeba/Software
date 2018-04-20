#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from sensor_msgs.msg import Image

class Template(object):
        def __init__(self, args):
                super(Template, self).__init__()
                self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.procesador)
                self.publisher = rospy.Publisher("/encerrarpatos", Image, queue_size=10)
                self.bridge = CvBridge()        

        def procesador(self, original):
                cv_img = self.bridge.imgmsg_to_cv2(original, "bgr8")

                mask = cv2.inRange(cv_img, (10,75,75), (45,255,255))

                kernel = np.ones((5,5),np.uint8)
                cv2.erode(mask, kernel, iterations=1)
                cv2.dilate(mask, kernel, iterations=1)

                
                image_out = cv2.bitwise_and(cv_img, cv_img, mask=mask)
                contour, hierarchy, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                
                x,y,w,h = cv2.boundingRect(contour)
                cv2.rectangle(image_out, (x,y), (x+w,y+h), (255,255,0), 2)                         
                print x,y,w,h
                actual = self.bridge.cv2_to_imgmsg(image_out, "bgr8")
                image_out = cv2.flip(image_out, 1)
                
                self.publisher.publish(actual)


def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
