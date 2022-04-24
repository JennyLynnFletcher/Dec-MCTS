#!/usr/bin/env python

import rospy
from threading import Thread
from std_msgs.msg import Empty
import time

import environment

if __name__ == '__main__':
    pub = rospy.Publisher('tick', Empty, queue_size=10)
    rospy.init_node('Main', anonymous=True)
    
    env = environment.Environment(15,15,(6,7), 10, render_interval=1)
    
    env.add_robot(0, (1,1), (5,5))
    env.add_robot(1, (1,1), (5,5))
    
    
    
    while True:
        pub.publish(Empty())
        time.sleep(0.2)
    