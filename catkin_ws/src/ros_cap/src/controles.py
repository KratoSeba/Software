#!/usr/bin/env python

import rospy #importar ros para python
#from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
#from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Joy
from duckietown_msgs.msg import Twist2DStamped


class Movimiento(object):
	def __init__(self):
		super(Movimiento, self).__init__()
		self.subscriber = rospy.Subscriber("/duckiebot/joy", Joy, self.callback)
		self.publisher = rospy.Publisher("/duckiebot/wheels_driver_node/car_cmd", Twist2DStamped, queue_size=10)
		self.twist = Twist2DStamped()
		self.speeds = [0.25, 0.50, 0.75, 1.00] # Velocidades
		self.indexSpeeds = 2 # Indice actual de velocidades



	def callback(self,msg):
		if all( [msg.axes[9], msg.axes[10]] ): # Presionar AMBOS STICKS para FORZAR ESCAPE
			self.twist.v = 0
			self.twist.omega = 0
			self.publisher.publish(self.twist) # Setea velocidades a 0 y detiene el codigo
			exit()

		if bool( msg.axes[2] ): self.indexSpeeds -= 1 # BOTON X DEGRADA VELOCIDAD
		if bool( msg.axes[1] ): self.indexSpeeds += 1 # BOTON B AUMENTA VELOCIDAD
		if self.indexSpeeds < 0: self.indexSpeeds = 0 # LIMITE INFERIOR
		if self.indexSpeeds > 3: self.indexSpeeds = 3 # LIMITE SUPERIOR

		frenoActivado = msg.buttons[5]          # {0, 1} | RB Button

		velocidad = msg.axes[1] * self.speeds[self.indexSpeeds]  # VELOCIDAD LINEAR. Ponderador default: 0.75
		velocidad *= not(frenoActivado)			# Freno activado => Velocidad Linear = 0

		omega = msg.axes[3] * 8         # VELOCIDAD ANGULAR

		self.twist.v = velocidad
		self.twist.omega = omega
			
		self.publisher.publish(self.twist)


def main():

	rospy.init_node("Controlador") #creacion y registro del nodo!

	obj = Movimiento()

	rospy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
