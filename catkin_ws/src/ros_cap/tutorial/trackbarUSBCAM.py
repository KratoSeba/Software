#!/usr/bin/env python

import cv2
import rospy #importar ros para python
from std_msgs.msg import Int32MultiArray # importar mensajes de ROS tipo String y tipo Int32


class Template(object):
	def __init__(self, args):
		super(Template, self).__init__()
		self.args = args
		self.publisher = rospy.Publisher("/trackbarUSBCAM", Int32MultiArray, queue_size=10)
		
		nothing = lambda x: x

		cv2.namedWindow('HSV Config + Erode + Dilate')
		cv2.createTrackbar('H MIN','HSV Config + Erode + Dilate',20,180,nothing)
		cv2.createTrackbar('S MIN','HSV Config + Erode + Dilate',130,255,nothing)
		cv2.createTrackbar('V MIN','HSV Config + Erode + Dilate',150,255,nothing)
		cv2.createTrackbar('Erode','HSV Config + Erode + Dilate',3,10,nothing)
		cv2.createTrackbar('Dilate','HSV Config + Erode + Dilate',6,10,nothing)
		cv2.createTrackbar('H MAX','HSV Config + Erode + Dilate',40,180,nothing)
		cv2.createTrackbar('S MAX','HSV Config + Erode + Dilate',255,255,nothing)
		cv2.createTrackbar('V MAX','HSV Config + Erode + Dilate',255,255,nothing)


	def actualizar(self):
		while 1:
			k = cv2.waitKey(1)
			if k == 27:
	
				break
			h1 = cv2.getTrackbarPos('H MIN','HSV Config + Erode + Dilate')
			s1 = cv2.getTrackbarPos('S MIN','HSV Config + Erode + Dilate')
			v1 = cv2.getTrackbarPos('V MIN','HSV Config + Erode + Dilate')
			erode = cv2.getTrackbarPos('Erode','HSV Config + Erode + Dilate')
			dilate = cv2.getTrackbarPos('Dilate','HSV Config + Erode + Dilate')
			h2 = cv2.getTrackbarPos('H MAX','HSV Config + Erode + Dilate')
			s2 = cv2.getTrackbarPos('S MAX','HSV Config + Erode + Dilate')
			v2 = cv2.getTrackbarPos('V MAX','HSV Config + Erode + Dilate')

			valores = [h1,s1,v1, erode, dilate, h2,s2,v2]
			print "TRACKBAR", valores
			self.publisher.publish(Int32MultiArray(data=valores))

def main():
	rospy.init_node('mytrackbar') #creacion y registro del nodo!

	obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	obj.actualizar() #llama al metodo publicar del objeto obj de tipo Template

if __name__ =='__main__':
	main()
