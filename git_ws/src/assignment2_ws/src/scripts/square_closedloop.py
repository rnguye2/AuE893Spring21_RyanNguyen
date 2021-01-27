#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
				                          Twist, queue_size=10)
				                          
		self.pose_publisher = rospy.Publisher('/turtle1/teleport_absolute',
				                          Pose,  queue_size=10)
				                    						
		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
		                                        Pose, self.update_pose)

		self.pose = Pose()
		self.rate = rospy.Rate(25)
		
	def update_pose(self, data):
		"""Callback function which is called when a new message of type Pose is
		received by the subscriber."""
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)

	def euclidean_distance(self, goal_pose):
		"""Euclidean distance between current pose and the goal."""
		return sqrt(pow((goal_pose.x - self.pose.x), 2) +
			    pow((goal_pose.y - self.pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

	def angular_vel(self, goal_pose, constant=6):
		"""See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
		return constant * (self.steering_angle(goal_pose) - self.pose.theta)

	def square(self):
		"""Moves the turtle in a square."""
		start_pose = Pose()
		
		
		start_pose.x = 5
		start_pose.y = 5
		
		self.pose_publisher.publish(start_pose)
		
		next_pose = Pose()
		goal_pose = Pose()
		
		pos = np.array([[8.55,5.55],[8.55,8.55], [5.55,8.55], [5.55,5.55]])

		next_pose.x = pos[0, 0]
		next_pose.y = pos[0, 1]
		
		goal_pose.x = 5.55
		goal_pose.y = 5.55 

		# Please, insert a number slightly greater than 0 (e.g. 0.01).
		distance_tolerance = 0.1
		#distance_tolerance = rospy.get_param('~tol')

		vel_msg = Twist()
		completed = False
		i = 0
		while completed == False:
			while self.euclidean_distance(next_pose) > distance_tolerance:

				# Porportional controller.
				# https://en.wikipedia.org/wiki/Proportional_control

				# Linear velocity in the x-axis.
				vel_msg.linear.x = self.linear_vel(next_pose)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = 0

				# Publishing our vel_msg
				self.velocity_publisher.publish(vel_msg)

				# Publish at the desired rate.
				self.rate.sleep()

			# Stopping our robot after the movement is over.
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			time_update = time.time()
			current_rotation_z = vel_msg.angular.z*(time.time()-time_update)
			time_update = time.time()
			while abs(current_rotation_z) < np.pi/2:
				vel_msg.angular.z = 0.5
				self.velocity_publisher.publish(vel_msg)
				current_rotation_z = vel_msg.angular.z*(time.time()-time_update)
			i += 1
			if i < len(pos[:, 0]):
				next_pose.x = pos[i, 0]
				next_pose.y = pos[i, 1]
				print(next_pose.y)
			else:
				completed = True

		# If we press control + C, the node will stop.
		rospy.spin()
			
if __name__ == '__main__':
	try:
		#Testing our function
		x = TurtleBot()
		x.square()
	except rospy.ROSInterruptException: pass
	
