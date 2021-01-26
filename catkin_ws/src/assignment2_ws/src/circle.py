#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class TurtleBot:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
				                          Twist, queue_size=10)
	def move_circle(r):

		move_cmd = Twist()
		move_cmd.linear.x = 1.0
		move_cmd.angular.z = 1.0


		rate = rospy.Rate(10)


if __name__ == '__main__':
	try:
		x = TurtleBot()
		r = 1
		x.move_circle()
	except rospy.ROSInterruptException:
		pass

