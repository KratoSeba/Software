#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy #importar ros para python
from std_msgs.msg import Int32MultiArray, Int32 # importar mensajes de ROS tipo String y tipo Int32
from sensor_msgs.msg import Image

class Template(object):
        def __init__(self, args):
                super(Template, self).__init__()
                self.subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, self.procesador)
                self.subscriberConfig = rospy.Subscriber("/trackbarUSBCAM", Int32MultiArray, self.update)
                self.publisher = rospy.Publisher("/detector", Image, queue_size=10)
                self.bridge = CvBridge()

                self.hsved = []

        def update(self, msg):
            self.hsved = msg.data

        def procesador(self, original):
                cv_img = self.bridge.imgmsg_to_cv2(original, "bgr8") # Pasar imagen de la camara a imagen cv2
                hsvImage = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsvImage, (self.hsved[0],self.hsved[1],self.hsved[2]), (self.hsved[5],self.hsved[6],self.hsved[7])) # Mascara para filtrar amarillo HSV
                
                kernel = np.ones((5,5),np.uint8) # Matriz para erosion y dilatacion
                erosion = cv2.erode(hsvImage, kernel, iterations=self.hsved[3])
                dilated = cv2.dilate(erosion, kernel, iterations=self.hsved[4])
                image_out = cv2.bitwise_and(dilated, cv_img, mask=mask) # Aplicar mascara

                _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Contorno y... jerarquia?

                rects = []
                for cnt in contours:
                	x,y,w,h = cv2.boundingRect(cnt) # x, y, width, height
                	if w * h > 1000:
                		print x,y,w,h
                		cv2.rectangle(image_out, (x,y), (x+w,y+h), (255,255,0), 2) # Crear rectangulo en la imagen original
                
                ###### Deben ir siempre juntos ######
                espejo = cv2.flip(image_out, 1) # Imagen modo espejo
                actual = self.bridge.cv2_to_imgmsg(espejo, "bgr8") # Imagen final de cv2 a imagen de "camara"?
                self.publisher.publish(actual) # Publicar la imagen


def main():
    rospy.init_node('detector') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
