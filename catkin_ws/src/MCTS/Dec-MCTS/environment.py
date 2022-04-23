import pygame
import rospy
from scipy import sparse
from std_msgs.msg import String
from geometry_msgs.msg import Point
from maze import Action
import maze_gen
from time import sleep, perf_counter
from threading import Thread

import decMCTS

white = (255, 255, 255)
grey = (100, 100, 100)
black = (0, 0, 0)


class Environment():
    def __init__(self, width, height, goal, num_robots, render_interval=0.5):
        self.width = width
        self.height = height
        self.walls = maze_gen.generate_maze(self.height, self.width)
        self.goal = goal
        self.render_interval = render_interval
        self.timestep = 0
        self.complete = False
        self.robot_list = {}
        self.grid_size = 20
        self.num_robots = num_robots

        pygame.init()
        self.gameDisplay = pygame.display.set_mode((1000, 1000))
        self.render()

    def get_goal(self):
        return self.goal

    def add_robot(self, robot_id, start_loc, goal_loc):
        '''
        Add robot with start location start_loc, goal goal_loc and pass self as env
        Add to self.robot_list
        '''
        def task():
            r = decMCTS.DecMCTS_Agent(robot_id=robot_id, start_loc=start_loc, goal_loc=goal_loc, env=self)
            r.control_loop()
        
        self.robot_list[robot_id] = (start_loc, goal_loc)
        #thread = Thread(target=task)
        #thread.start()
        #thread.join()   
        task()
        print("robot_id: ", robot_id)
        

    def get_walls_from_loc(self, loc):
        '''
        return a dictionary of N,E,S,W and whether there
        is a wall in that direction from loc
        '''
        x, y = loc
        return {
            Action.UP:    self.walls[x, y - 1] == 0,
            Action.DOWN:  self.walls[x, y + 1] == 0,
            Action.LEFT:  self.walls[x - 1, y] == 0,
            Action.RIGHT: self.walls[x + 1, y] == 0
        }

    def render(self):
        '''
        render stuff
        requires testing
        '''

        self.gameDisplay.fill(grey)

        for path_y, path_x, _ in zip(*sparse.find(self.walls)):
            rect = (
                (path_x - 0.5) * self.grid_size,
                (path_y - 0.5) * self.grid_size,
                self.grid_size,
                self.grid_size
            )
            pygame.draw.rect(self.gameDisplay, white, rect, width=0)

        for _, pos in self.robot_list:
            pygame.draw.circle(self.gameDisplay, black, pos * self.grid_size, 0.3 * self.grid_size)
        pygame.display.update()

    def update_loc(self, loc_msg, robot_id):
        '''
        Callback function for robot locations
        Update locations of robots for rendering
        '''
        x = loc_msg.x
        y = loc_msg.y
        self.robot_list[robot_id] = (x,y)
        self.render()

    def listener(self):
        '''
        Implement listener code, call to update_loc()
        Implement timer listener at frequency 1/render_interval to call to render()
        '''
        rospy.init_node('Environment', anonymous=True)
        for (robot_id, _) in self.robot_list:
            rospy.Subscriber('robot_loc_' + str(robot_id), Point, self.update_loc, robot_id)
