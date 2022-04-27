#!/usr/bin/env python3
import threading
import os
import random

import rospy
from threading import Thread
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point

import time
import pygame

import environment
import decMCTS


def call_update(x):
    robot, is_execute_iteration = x
    robot.update(is_execute_iteration)
    return robot

def main(comms_aware=True, num_robots=3, seed=0, name="default"):
    rospy.init_node('Main', anonymous=True)

    if not os.path.exists("output/" + name):
        os.makedirs("output/" + name)

    width = 11
    height = 11
    
    random.seed(seed)
    goal = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
    print("Goal: ", goal)
    env = environment.Environment(width, height, goal, num_robots, render_interval=1, seed=0)

    robot_start_locations = []
    for _ in range(num_robots):
        loc = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
        while loc == goal:
            loc = (random.randrange(0, width // 2) * 2 + 1, random.randrange(0, height // 2) * 2 + 1)
        robot_start_locations.append(loc)
    print("Start locations: " + str(robot_start_locations))
    robots = []
    random.seed()
    for robot_id, start_location in enumerate(robot_start_locations):
        env.add_robot(robot_id, start_location, goal)
        rospy.Subscriber("robot_probs", String, lambda x: robots[-1].reception_queue.append(x))

    env.set_up_listener()

    for robot_id, start_location in enumerate(robot_start_locations):
        robots.append(decMCTS.DecMCTS_Agent(robot_id=robot_id, start_loc=start_location, goal_loc=goal, env=env,
                                            comms_drop="distance", comms_drop_rate=0.9,
                                            comms_aware_planning=comms_aware))

    i = -1
    frames = 0
    pygame.display.update()
    pygame.image.save(env.gameDisplay, "output/" + name + "/frame_" + str(frames) + ".jpg")

    complete = False
    while not complete:
        pygame.display.update()
        is_execute_iteration = ((i % 2) == 0)
        i += 1
        threads = []


        # with multiprocessing.Pool(num_robots) as p:
        #     robots = p.map(call_update,[(r,is_execute_iteration) for r in robots])

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
        pygame.display.update()
        if is_execute_iteration:
            frames += 1
            pygame.image.save(env.gameDisplay, "output/" + name + "/frame_" + str(frames) + ".jpg")

    pygame.quit()
    with open('./results.txt', 'a') as f:
        f.write("Comms_aware: " + str(comms_aware) + ", num_robots: " + str(num_robots) + ", Iterations: " + str(i))


try:
    for i in range(10):
        main(comms_aware=True, num_robots=5, seed=i, name="11x11_comms_" + str(i))
        main(comms_aware=False, num_robots=5, seed=i, name="11x11_nocoms_" + str(i))
except SystemExit:
    pygame.quit()
