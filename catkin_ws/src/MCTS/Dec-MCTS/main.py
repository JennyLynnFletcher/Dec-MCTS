#!/usr/bin/env python

import rospy
from threading import Thread
from std_msgs.msg import Empty
import time

import environment

if __name__ == '__main__':
    env = environment.Environment(15,15,(6,7), 10, render_interval=1)
    
    env.add_robot(0, (1,2), (4,4))
    env.add_robot(1, (1,2), (4,4))
    
    pub = rospy.Publisher('tick', Empty, queue_size=10)
    rospy.init_node('Main', anonymous=True)
    
    while True:
        time.sleep(1)
        pub.publish(Empty())
        time.sleep(5)
    