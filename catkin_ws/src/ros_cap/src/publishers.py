#!/usr/bin/env python

import rospy #importar ros para python
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist


class Contador(object):
	def __init__(self, args):
		super(Contador, self).__init__()
		self.publisher = rospy.Publisher("/chatter", Int32, queue_size=10)		
                self.numero = 0

	def publicar(self):
                while 1:
                        msg = Int32()
                        msg.data = self.numero
                        self.publisher.publish(msg)
                        self.numero += 1
                        rospy.sleep(0.5)

def main():
	rospy.init_node('Contador') #creacion y registro del nodo!

	obj = Contador('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	obj.publicar() #llama al metodo publicar del objeto obj de tipo Template

	#rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
