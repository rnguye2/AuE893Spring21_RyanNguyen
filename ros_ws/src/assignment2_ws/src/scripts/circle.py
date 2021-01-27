#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

class TurtleBot:

	def __init__(self):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node('turtlebot_controller', anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
				                          Twist, queue_size=10)
				                    						
	
		self.rate = rospy.Rate(10)


	def circle(self):
		time_duration = 5
		time_start = time.time()
		
		while time.time() < time_start + time_duration:
			move_cmd = Twist()
			move_cmd.linear.x = 10.0
			move_cmd.angular.z = move_cmd.linear.x/rospy.get_param('~r')



			self.velocity_publisher.publish(move_cmd)

if __name__ == '__main__':
	try:
		x = TurtleBot()
		x.circle()
	except rospy.ROSInterruptException:
		pass

