#! /usr/bin/python

import sys
from math import sin, cos

import rospy
import tf2_ros
from rospy import loginfo as log
from turtlesim.msg import Pose

BASE_FRAME = 'world'
MAP_FRAME = 'map'
TURTLE_FRAME = lambda turtle: f'turtle_{turtle}'

broadcaster: tf2_ros.TransformBroadcaster = None


def send_map_transform(x=5.5, y=5.5, z=0, w=1):  # rename args
    assert z * z + w * w == 1

    static_br = tf2_ros.StaticTransformBroadcaster()
    map_t = tf2_ros.TransformStamped()
    map_t.header.stamp = rospy.Time.now()
    map_t.header.frame_id = BASE_FRAME
    map_t.child_frame_id = MAP_FRAME

    map_t.transform.translation.x = x
    map_t.transform.translation.y = y

    map_t.transform.rotation.z = z
    map_t.transform.rotation.w = w

    static_br.sendTransform(map_t)


def publish_transform(pose: Pose, t: tf2_ros.TransformStamped):
    t.header.stamp = rospy.Time.now()
    t.transform.translation.x = pose.x
    t.transform.translation.y = pose.y
    t.transform.rotation.z = sin(pose.theta / 2)
    t.transform.rotation.w = cos(pose.theta / 2)

    global broadcaster
    broadcaster.sendTransform(t)


def main():
    argv = rospy.myargv(argv=sys.argv)[1:]

    rospy.init_node("state_publisher")

    log('State publisher started')

    send_map_transform()
    log('Sent map frame.')

    global broadcaster
    broadcaster = tf2_ros.TransformBroadcaster()

    for turtle in argv:
        log(f'Started publishing transform for  {TURTLE_FRAME(turtle)}.')

        t = tf2_ros.TransformStamped()
        t.header.frame_id = BASE_FRAME
        t.child_frame_id = TURTLE_FRAME(turtle)
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0

        rospy.Subscriber(f'/{turtle}/pose', Pose, publish_transform, t)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException:
        print('Failed.')
