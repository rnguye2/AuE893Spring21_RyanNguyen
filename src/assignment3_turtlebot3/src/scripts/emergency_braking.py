#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurtleBot:
    def __init__(self, node_name, topic):
        rospy.init_node(node_name, anonymous=True)

        # Publisher which will publish to the given topic.
        self.velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)

        # A subscriber to the topic '/scan'.
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.update)

        # Init the velocity message
        self.vel_msg = Twist()

        # Init the publish rate
        self.rate = rospy.Rate(10)

        # Init the distance to the wall
        self.distance = 100000

    def update(self, msg):
        # values at 0 degree
        self.distance = msg.ranges[0]

    def move(self, threshold=0.5):
        """ Move the turtlebot. """
        self.vel_msg.linear.x = 0.3

        while self.distance > threshold:
            # Publish the vel_msg
            self.velocity_publisher.publish(self.vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        self.vel_msg.linear.x = 0
        # Publish the vel_msg
        self.velocity_publisher.publish(self.vel_msg)
        # Publish at the desired rate.
        self.rate.sleep()

        # If control + C is pressed, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot('turtlebot_controller', '/cmd_vel')
        x.move()
    except rospy.ROSInterruptException:
        pass
