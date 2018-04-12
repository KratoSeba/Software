#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped


class Movimiento(object):
	def __init__(self):
		super(Movimiento, self).__init__()
		self.subscriber = rospy.Subscriber("/duckiebot/joy",Joy,self.callback)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd",Twist2DStamped,queue_size=10)
		self.twist = Twist2DStamped()




	def callback(self,msg):
		velocidad = msg.axes[1]	
		if velocidad > 0:
			self.twist.v = 1
			print 1
		elif velocidad < 0:
			self.twist.v = -1
			print 2
		else:
			self.twist.v = 0
			print 3

		self.publisher.publish(self.twist)

		print velocidad


def main():
	rospy.init_node('test') #creacion y registro del nodo!

	obj = Movimiento() # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()