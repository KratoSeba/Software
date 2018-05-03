#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy #importar ros para python
from std_msgs.msg import Int32MultiArray, Int32 # importar mensajes de ROS tipo String y tipo Int32
from sensor_msgs.msg import Image

class Detector(object):
        def __init__(self):
                super(Detector, self).__init__()
                self.subscriber = rospy.Subscriber("/duckiebot/camera_node/image/rect", Image, self.procesador) # Nodo camara
                #self.subscriberConfig = rospy.Subscriber("/trackbarUSBCAM", Int32MultiArray, self.update) # Nodo que configura erode, dilate y hsv
                self.publisher = rospy.Publisher("/detector", Image, queue_size=10) # Publicador de la nueva imagen
                self.bridge = CvBridge()

                self.hsved = [20,140,110,1,1,40,225,225] # Aqui se reciben los valores del trackbar

        def update(self, msg): # Del nodo trackbar
            self.hsved = msg.data

        def procesador(self, original):
                cv_img = self.bridge.imgmsg_to_cv2(original, "bgr8") # Pasar imagen de la camara a imagen cv2
                hsvImage = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

                mask = cv2.inRange(hsvImage, (self.hsved[0],self.hsved[1],self.hsved[2]), (self.hsved[5],self.hsved[6],self.hsved[7])) # Mascara para filtrar amarillo HSV
                
                kernel = np.ones((5,5),np.uint8) # Matriz para erosion y dilatacion
                erosion = cv2.erode(mask, kernel, iterations=self.hsved[3])
                dilated = cv2.dilate(erosion, kernel, iterations=self.hsved[4])
                image_out = cv2.bitwise_and(cv_img, cv_img, mask=dilated) # Aplicar mascara

                _, contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Contornos externos, Aproximacion simple

                rects = []
                for cnt in contours:
                	x,y,w,h = cv2.boundingRect(cnt) # x, y, width, height

                	fx = 170.8734424022098
                	fy = 172.04387162468095
                	cx = 157.0965555137086
                	cy = 113.2325768841192
                	dPx = 3.5
                	dPy = 3

                	z = fy * dPy / h
                	xx = ( (x+w/2) - cx ) * z / fx
                	yy = ( (y+h/2) - cy ) * z / fy

                	print z
                	
                	if w * h >= 250: # Area minima del rectangulo = 1000 pixeles
                		#print (x, y)
                		cv2.rectangle(image_out, (x,y), (x+w,y+h), (255,255,0), 2) # Crear rectangulo en la imagen original

                # Ver mascara: image_out | Ver imagen original: cv-img #

                ###### Deben ir siempre juntos ######
                espejo = cv2.flip(cv_img, 1) # Imagen modo espejo
                actual = self.bridge.cv2_to_imgmsg(image_out, "bgr8") # Imagen final de cv2 a imagen de "camara"?
                self.publisher.publish(actual) # Publicar la imagen


def main():
    rospy.init_node('detector') #creacion y registro del nodo!

    obj = Detector() # Crea un objeto del tipo Detector, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Detector

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
