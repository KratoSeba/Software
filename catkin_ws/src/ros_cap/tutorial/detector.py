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
                cv_img = self.bridge.imgmsg_to_cv2(original, "bgr8") # Pasar imagen de la camara a imagen cv2

                mask = cv2.inRange(cv_img, (20,100,100), (50,255,255)) # Mascara para filtrar amarillo

                kernel = np.ones((5,5),np.uint8) # Matriz para erosion y dilatacion
                cv2.erode(mask, kernel, iterations=5)
                cv2.dilate(mask, kernel, iterations=5)

                
                image_out = cv2.bitwise_and(cv_img, cv_img, mask=mask) # Aplicar mascara
                contours, hierarchy, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # Contorno y... jerarquia?
                
                x,y,w,h = cv2.boundingRect(contours) # x, y, width, height
                cv2.rectangle(cv_img, (x,y), (x+w,y+h), (255,255,0), 2) # Crear rectangulo en la imagen original
                
                ###### Deben ir siempre juntos ######
                image_out = cv2.flip(image_out, 1) # Imagen modo espejo
                actual = self.bridge.cv2_to_imgmsg(cv_img, "bgr8") # Imagen final de cv2 a imagen de "camara"?
                self.publisher.publish(actual) # Publicar la imagen


def main():
    rospy.init_node('test') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

    #objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
    main()
