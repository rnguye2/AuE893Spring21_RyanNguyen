#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleBot:

	def __init__(self, name, topic):
		# Creates a node with name 'turtlebot_controller' and make sure it is a
		# unique node (using anonymous=True).
		rospy.init_node(name, anonymous=True)

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher(topic,
				                          Twist, queue_size=10)
				                    						
		self.move_cmd = Twist()
		self.rate = rospy.Rate(10)


	def circle(self):
		while not rospy.is_shutdown():
			self.move_cmd.linear.x = 0.5
			self.move_cmd.angular.z = 0.5

			self.velocity_publisher.publish(self.move_cmd)
			
			self.rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	try:
		x = TurtleBot('turtlebot_controller', '/cmd_vel')
		x.circle()
	except rospy.ROSInterruptException:
		pass

