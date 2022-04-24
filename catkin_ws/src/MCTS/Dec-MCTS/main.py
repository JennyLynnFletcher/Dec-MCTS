#!/usr/bin/env python
import random

import rospy
from threading import Thread
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point

import time
from threading import Thread

import environment
import decMCTS

if __name__ == '__main__':
    rospy.init_node('Main', anonymous=True)

    width = 31
    height=31
    num_robots=2

    goal = (15, 15)
    env = environment.Environment(width, height, goal, num_robots, render_interval=1)

    robot_start_locations = []
    for _ in range(num_robots):
        robot_start_locations.append((random.randrange(0,width//2)*2+1, random.randrange(0,height//2)*2+1))
    print("Start locations: "+str(robot_start_locations))
    robots = []

    for robot_id, start_location in enumerate(robot_start_locations):
        env.add_robot(robot_id, start_location, goal)
        robots.append(decMCTS.DecMCTS_Agent(robot_id=robot_id, start_loc=start_location, goal_loc=goal, env=env))
        rospy.Subscriber("robot_obs", String, lambda x: robots[-1].reception_queue.append(x))

    env.set_up_listener()

    for i, loc in enumerate(robot_start_locations):
        msg = Point(x=loc[0], y=loc[1])
        rospy.Publisher('robot_loc_' + str(i), Point, queue_size=10).publish(msg)

    i = -1
    while True:
        is_execute_iteration = ((i % 2) == 0)
        i += 1
        print(i)
        threads = []
        for r in robots:
            thread = Thread(target=r.update, args=[is_execute_iteration])
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()
