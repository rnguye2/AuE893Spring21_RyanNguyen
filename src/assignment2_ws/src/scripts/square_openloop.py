#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np

class TurtleBot:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
				                          Twist, queue_size=10)
				                    						
	
		self.rate = rospy.Rate(30)
	def square(self):
		
		
		vel_msg = Twist()

		#Loop to move the turtle in an specified distance
		time_duration = 80
		time_start = time.time()
		time_update = time.time()
		vel_msg.linear.x = 0.2
		#Publish the velocity
		self.velocity_publisher.publish(vel_msg)
		count = 0
		while time.time() < time_start + time_duration:
			current_distance_x = vel_msg.linear.x*(time.time()-time_update)
			current_rotation_z = vel_msg.angular.z*(time.time()-time_update)
			if abs(current_distance_x) >= 2:
				vel_msg.linear.x = 0
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0.2
				time_update = time.time()
			elif abs(current_rotation_z) >= np.pi/2:
				vel_msg.linear.x = 0.2
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0
				time_update = time.time()
				
			#Publish the velocity
			self.velocity_publisher.publish(vel_msg)
			
if __name__ == '__main__':
	try:
		#Testing our function
		x = TurtleBot()
		x.square()
	except rospy.ROSInterruptException: pass
	
