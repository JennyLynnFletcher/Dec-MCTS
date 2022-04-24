#!/usr/bin/env python

import rospy
from threading import Thread
from std_msgs.msg import Empty
import time

import environment

if __name__ == '__main__':
    pub = rospy.Publisher('tick', Empty, queue_size=10)
    rospy.init_node('Main', anonymous=True)

    goal = (5, 5)
    env = environment.Environment(15, 15, goal, 10, render_interval=1)

    env.add_robot(0, (1, 1), goal)
    env.add_robot(1, (1, 1), goal)

    while True:
        pub.publish(Empty())
        time.sleep(0.2)
    