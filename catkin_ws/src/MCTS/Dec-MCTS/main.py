#!/usr/bin/env python
import sys
import threading
import os
import random

import rospy
from threading import Thread
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point
import fcntl

import time
import pygame

import environment
import decMCTS
import logging


def call_update(x):
    robot, is_execute_iteration = x
    robot.update(is_execute_iteration)
    return robot


def main(comms_aware=True, num_robots=3, seed=0, name="default", out_of_date_timeout=None):
    rospy.init_node('Main_' + str(name), anonymous=True)

    if not os.path.exists("output/" + name):
        os.makedirs("output/" + name)

    print(rospy.get_namespace())

    width = 11
    height = 11

    random.seed(seed)
    goal = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
    print("Goal: ", goal)
    env = environment.Environment(width, height, goal, num_robots, render_interval=1, seed=seed, name=name)

    robot_start_locations = []
    for _ in range(num_robots):
        loc = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
        while loc == goal:
            loc = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
        robot_start_locations.append(loc)
    print("Start locations: " + str(robot_start_locations))
    robots = []
    for robot_id, start_location in enumerate(robot_start_locations):
        env.add_robot(robot_id, start_location, goal)
        rospy.Subscriber('robot_obs_'+name, String, lambda x: robots[-1].reception_queue.append(x))

    env.set_up_listener()

    for robot_id, start_location in enumerate(robot_start_locations):
        robots.append(decMCTS.DecMCTS_Agent(robot_id=robot_id, start_loc=start_location, goal_loc=goal, env=env,
                                            comms_drop="distance", comms_drop_rate=0.9,
                                            comms_aware_planning=comms_aware,
                                            out_of_date_timeout=out_of_date_timeout,
                                            name=name))

    i = -1
    frames = 0
    pygame.display.update()
    pygame.image.save(env.gameDisplay, "output/" + name + "/frame_" + str(frames) + ".jpg")

    complete = False
    while not complete:
        is_execute_iteration = ((i % 2) == 0)
        i += 1
        threads = []

        for r in robots:
            thread = threading.Thread(target=r.update, args=(is_execute_iteration,))
            threads.append(thread)
        random.shuffle(threads)
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        complete = True
        for r in robots:
            complete = complete and r.complete
        if is_execute_iteration:
            frames += 1
            pygame.display.update()
            pygame.image.save(env.gameDisplay, "output/" + name + "/frame_" + str(frames) + ".jpg")

    pygame.quit()
    with open('./results.txt', 'a') as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        f.write("Seed: " + str(seed) + " comms_aware: " + str(comms_aware) + " num_robots: " + str(
            num_robots) + " Iterations: " + str(i) + " Times forgot other agent: "
                + str([agent.times_removed_other_agent for agent in robots]) + "\n")
        fcntl.flock(f, fcntl.LOCK_UN)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        try:

            print(sys.argv)

            name = sys.argv[1]

            seed = int(sys.argv[2])
            num_robots = int(sys.argv[3])
            maze_width = int(sys.argv[4])
            comms = sys.argv[5].strip() == "True"
            out_of_date_timeout = int(sys.argv[6]) if int(sys.argv[6]) > 0 else None

            fullname = name + "__" + str(maze_width) + "x" + str(maze_width) + "__comms_" + str(
                     comms) + "__timeout_" + str(
                     out_of_date_timeout) + "__seed_" + str(seed)
            logging.basicConfig(filename='./MyLog'+fullname+'.log', level=logging.DEBUG)


            main(comms_aware=comms, num_robots=num_robots, seed=seed,
                 name=fullname,
                 out_of_date_timeout=out_of_date_timeout)
        except Exception as e:
            print(e)

    else:
        for i in range(10):
            main(comms_aware=True, num_robots=5, seed=i, name="11x11_comms" + str(i))
            main(comms_aware=False, num_robots=5, seed=i, name="11x11_nocomms" + str(i))
