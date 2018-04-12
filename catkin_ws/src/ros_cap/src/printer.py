#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class Printer(object):
	def __init__(self, args):
		super(Printer, self).__init__()
           	self.subscriber = rospy.Subscriber("/chatter", Int32, self.callback)
                
	def callback(self,msg):
                rospy.loginfo(msg.data)


def main():
	rospy.init_node('Printer') #creacion y registro del nodo!

	obj = Printer('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
