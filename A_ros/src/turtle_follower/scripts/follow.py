#! /usr/bin/env python

import sys
from math import atan2, pi, sin

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from rospy import loginfo as log

TURTLE_FRAME = lambda turtle: f'turtle_{turtle}'
TURTLE_CMD_VEL = lambda turtle: f'/{turtle}/cmd_vel'

VEL_MAX = 1.5

MIN_DIST_SQ = 1.5 ** 2


def main():
    argv = rospy.myargv(argv=sys.argv)[1:]
    rospy.init_node("state_publisher")
    log('Follower started.')

    if len(argv) != 2:
        rospy.logerr("Invalid arguments.")
        return 1
    turtle, goal = argv

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    publisher = rospy.Publisher(TURTLE_CMD_VEL(turtle), Twist, queue_size=1)

    vel = Twist()

    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        try:
            t = tfBuffer.lookup_transform(TURTLE_FRAME(turtle), TURTLE_FRAME(goal), rospy.Time()).transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        if t.translation.x ** 2 + t.translation.y ** 2 > MIN_DIST_SQ:
            # TODO Change this thing.
            theta = atan2(t.translation.y, t.translation.x)
            vr = 1 + 2 * sin(theta / 2)
            vl = 1 - 2 * sin(theta / 2)

            vel.linear.x = VEL_MAX * (vr + vl) / 2
            vel.angular.z = (vr - vl) / 2 * pi
        else:
            vel.linear.x = 0
            vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException:
        print('Failed.')
