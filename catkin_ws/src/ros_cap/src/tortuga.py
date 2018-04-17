#!/usr/bin/env python

from time import sleep
import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class ControlTortuga(object):
	def __init__(self, args):
		super(ControlTortuga, self).__init__()
		self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
		self.twist = Twist()
                PI = 3.1415926535897

                velocidad = PI / 6
                angulo = PI / 2

                t0 = rospy.Time.now().to_sec()
                anguloActual = 0
                self.twist.angular.z = velocidad

                while anguloActual < angulo:
                        self.publisher.publish(self.twist)
                        t1 = rospy.Time.now().to_sec()
                        anguloActual = velocidad*(t1-t0)

                self.twist.angular.z = 0
                self.publisher.publish(self.twist)
                        

def main():
	rospy.init_node('ControlTortuga') #creacion y registro del nodo!

	obj = ControlTortuga('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
