#! /usr/bin/env python
import rospy
import std_msgs


def callback(message):
    print(message)
    rospy.loginfo('I feel welcomed.')


def main():
    sub = rospy.Subscriber('/welcome_message', std_msgs.msg.String, callback)
    rospy.init_node('welcomed', anonymous=False)

    print('Ready to receive.')

    rospy.spin()


if __name__ == '__main__':
    main()
