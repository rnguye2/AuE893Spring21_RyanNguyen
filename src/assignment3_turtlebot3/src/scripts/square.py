#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class TurtleBot:
    def __init__(self, name, topic):
        rospy.init_node(name, anonymous=True)

        # Publisher which will publish to the given topic.
        self.velocity_publisher = rospy.Publisher(topic, Twist, queue_size=10)

        # Init the velocity message
        self.vel_msg = Twist()

        # Init the publish rate
        self.rate = rospy.Rate(10)

    def sqaure(self):
        """ Move the turtlebot in a square. """
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0


        square_size = 2
        velocity = 0.3
        angular_velocity = 0.3

        # Keep moving until shutdown
        while not rospy.is_shutdown():
            # Set moving speed
            self.vel_msg.linear.x = velocity
            self.vel_msg.angular.z = 0

            current_distance = 0
            # Get the current time for distance calculus
            t0 = rospy.Time.now().to_sec()

            # Move forward
            while current_distance < square_size:
                # Publish the vel_msg
                self.velocity_publisher.publish(self.vel_msg)
                
                # Calculate the current distance
                t1 = rospy.Time.now().to_sec()
                current_distance = velocity * (t1 - t0)

                # Publish at the desired rate.
                self.rate.sleep()

            # Make a 90 degrees turn
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = angular_velocity

            current_angle = 0
            t0 = rospy.Time.now().to_sec()

            while current_angle < 3.14159265358979 / 2:
                # Publish the vel_msg
                self.velocity_publisher.publish(self.vel_msg)
                # Calculate the current distance
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_velocity * (t1 - t0)
                # Publish at the desired rate.
                self.rate.sleep()



        # If control + C is pressed, the node will stop.
        rospy.spin()


if __name__ == '__main__':
    try:
        x = TurtleBot('turtlebot_controller', '/cmd_vel')
        x.sqaure()
    except rospy.ROSInterruptException:
        pass
