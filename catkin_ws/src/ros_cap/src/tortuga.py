#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class Move(object):
	def __init__(self, args):
		super(Move, self).__init__()
		self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
		self.subscriber = rospy.Subscriber("/chatter", Int32, self.mover)
		self.twist = Twist()

	def twist_publicar(self, vel):
		self.twist.linear.x = vel
		self.publisher.publish(self.twist)

	def mover(self, msg):
		num = msg.data
		if num % 2 == 0:
			vel = 1
		else:
			vel = -1
		self.twist_publicar(vel)


def main():
	rospy.init_node('Move') #creacion y registro del nodo!

	obj = Move('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
