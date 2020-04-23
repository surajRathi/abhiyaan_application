#! /usr/bin/env python
# Here we turn and move simultaneously
# This isn't the best method, as we may want to move less fast while turning. But this seems better than 4.py

from math import atan2, copysign

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

pub = None

speed = 1
omega = 0.5

abhiyaan_position = np.zeros(2)
turtle1_position = np.zeros(2)
turtle1_angle = 0
t = Twist()


def abhiyaan_pose_setter(pose_new):
    global abhiyaan_position
    abhiyaan_position[0] = pose_new.x
    abhiyaan_position[1] = pose_new.y


def turtle1_pose_setter(pose_new):
    global turtle1_position, turtle1_angle
    turtle1_position[0] = pose_new.x
    turtle1_position[1] = pose_new.y
    turtle1_angle = pose_new.theta


def send_velocity(v, w):
    global pub, t
    t.linear.x = v
    t.angular.z = w
    pub.publish(t)


def distance():
    global abhiyaan_position, turtle1_position
    return np.linalg.norm(abhiyaan_position - turtle1_position)


def relative_angle():
    global abhiyaan_position, turtle1_position, turtle1_angle
    return atan2(*((abhiyaan_position - turtle1_position)[::-1])) - turtle1_angle


def main():
    global pub, speed, omega, abhiyaan_position, turtle1_position, turtle1_angle

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/abhiyaan/pose', Pose, abhiyaan_pose_setter)
    rospy.Subscriber('/turtle1/pose', Pose, turtle1_pose_setter)

    rospy.init_node('turtle1_mover')

    rospy.sleep(1)

    angle = relative_angle()
    print(f'Current angle {angle} and distance {distance()}.')

    rate = rospy.Rate(60)
    while (not rospy.is_shutdown()) and (distance() > 2):
        send_velocity(speed, copysign(omega, relative_angle()))
        rate.sleep()

    send_velocity(0, 0)

    print('Reached target position.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
