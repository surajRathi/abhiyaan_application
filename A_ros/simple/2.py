#! /usr/bin/env python
import rospy
import std_msgs


def main():
    pub = rospy.Publisher('/welcome_message', std_msgs.msg.String, queue_size=1)
    rospy.init_node('welcomer', anonymous=False)

    print('Publishing welcome message.')
    pub.publish('Welcome to Abhiyaan')

    rospy.loginfo('I have welcomed.')


if __name__ == '__main__':
    main()
