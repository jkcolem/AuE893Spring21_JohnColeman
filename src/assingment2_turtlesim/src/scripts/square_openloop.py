#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg  import Twist
from math import pi

def move(speed,distance,turn_angle):
    rospy.init_node('turtlebot_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Linear velocity in the x-axis.
    vel_msg.linear.x=speed

    # Setting the current time for distance calculus
    t0=float(rospy.Time.now().to_sec())
    current_distance=0

    while (current_distance<distance):

        velocity_publisher.publish(vel_msg)
        tf=float(rospy.Time.now().to_sec())
        current_distance=speed*(tf-t0)
       
    # Stoping linear velocity in the x-axis and starting angular velocity in the z-axis
    vel_msg.linear.x=0
    vel_msg.angular.z=speed

    # Setting the current time for distance calculus
    t0=float(rospy.Time.now().to_sec())
    current_angle=0
    relative_angle=float(turn_angle*pi/180)

    while (current_angle<relative_angle):
        velocity_publisher.publish(vel_msg)
        tf=float(rospy.Time.now().to_sec())
        current_angle=speed*(tf-t0)
        
    # Forcing robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

def square():
    # Draws each side of the square
    for _ in range(4):
        move(0.2,2,90)
        
if __name__ == '__main__':
    try:
        square()
    except rospy.ROSInterruptException: pass

