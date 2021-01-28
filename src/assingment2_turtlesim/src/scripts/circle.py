#!/usr/bin/env python3
import rospy
from geometry_msgs.msg  import Twist
from math import pi

def rotate():
    rospy.init_node('turtlebot_controller', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Intial vales for circle
    # Agle needs to be greater than 360
    speed=1.5
    angle=400
    clockwise=False
    relative_angle=float(angle*pi/180)

    # Linear velocity in the x-axis.
    vel_msg.linear.x=speed
 
    # Checking if our movement is CW or CCW
    if (clockwise==True):
        vel_msg.angular.z=-abs(speed)
    else:
        vel_msg.angular.z=abs(speed)
    
    # Setting the current time for distance calculus
    t0=float(rospy.Time.now().to_sec())
    current_angle=0

    while(current_angle<relative_angle):
        velocity_publisher.publish(vel_msg)
        tf=float(rospy.Time.now().to_sec())
        current_angle=speed*(tf-t0)

    # Forcing robot to stop
    vel_msg.angular.z = 0
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException: pass
