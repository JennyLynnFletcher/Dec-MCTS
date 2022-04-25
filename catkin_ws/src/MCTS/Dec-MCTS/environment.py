import random
from enum import Enum

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


class Colour(Enum):
    Black = (0, 0, 0)
    Grey = (100, 100, 100)
    White = (255, 255, 255)
    Red = (168, 22, 0)
    Blue = (22, 0, 168)
    Green = (0, 168, 22)
    Cyan = (0, 168, 168)
    Yellow = (200, 200, 0)
    Magenta = (168, 0, 168)
    Orange = (200, 90, 0)
    Purple = (100, 0, 168)
    LightRed = (200, 100, 100)
    LightBlue = (100, 100, 200)
    LightGreen = (100, 200, 100)
    LightCyan = (100, 200, 200)
    LightYellow = (200, 200, 100)
    LightMagenta = (200, 100, 200)
    LightPurple = (168, 100, 255)
    LightOrange = (255, 168, 100)
    GoalGreen = (0, 255, 0)


colour_order = [Colour.Red, Colour.Blue, Colour.Green, Colour.Cyan, Colour.Yellow, Colour.Magenta, Colour.Orange,
                Colour.Purple, Colour.LightRed, Colour.LightBlue, Colour.LightGreen, Colour.LightCyan,
                Colour.LightYellow, Colour.LightMagenta, Colour.LightOrange, Colour.LightPurple]


class Environment():
    def __init__(self, width, height, goal, num_robots, render_interval=0.5):
        self.width = width + 2
        self.height = height + 2
        self.walls = maze_gen.generate_maze(height, width)
        self.goal = goal
        self.render_interval = render_interval
        self.timestep = 0
        self.complete = False
        self.robot_list = {}
        self.robot_colors = {}
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

        self.robot_list[robot_id] = (start_loc, goal_loc)
        self.robot_colors[robot_id] = colour_order[len(self.robot_list) - 1 % len(colour_order)]
        print("robot_id: ", robot_id)
        self.render()
        

    def get_walls_from_loc(self, loc):
        '''
        return a dictionary of N,E,S,W and whether there
        is a wall in that direction from loc
        '''
        x, y = loc
        return {
            Action.LEFT:    self.walls[y, x - 1] == 0,
            Action.RIGHT:  self.walls[y, x + 1] == 0,
            Action.UP:  self.walls[y - 1, x] == 0,
            Action.DOWN: self.walls[y + 1, x] == 0
        }

    def render(self):
        '''
        render stuff
        requires testing
        '''
        self.gameDisplay.fill(Colour.Grey.value)
        print("Rendering...")
        for path_y, path_x, _ in zip(*sparse.find(self.walls)):
            rect = (
                (path_x - 0.5) * self.grid_size,
                (path_y - 0.5) * self.grid_size,
                self.grid_size,
                self.grid_size
            )
            pygame.draw.rect(self.gameDisplay, Colour.White.value, rect, width=0)
        pygame.draw.rect(
            self.gameDisplay,
            Colour.GoalGreen.value,
            (
                (self.goal[0] - 0.5) * self.grid_size,
                (self.goal[1] - 0.5) * self.grid_size,
                self.grid_size,
                self.grid_size
            )
            , width=0)

        for robot_id, ((x, y), _) in self.robot_list.items():
            pygame.draw.circle(surface=self.gameDisplay,
                               color=self.robot_colors[robot_id].value,
                               center=(x * self.grid_size,y*self.grid_size),
                               radius=0.3 * self.grid_size)
        #pygame.display.update()

    def update_loc(self, loc_msg, robot_id):
        '''
        Callback function for robot locations
        Update locations of robots for rendering
        '''
        x = loc_msg.x
        y = loc_msg.y
        self.robot_list[robot_id] = ((x,y), self.robot_list[robot_id][1])
        print("updated robot "+str(robot_id))
        self.render()

    def set_up_listener(self):
        '''
        Implement listener code, call to update_loc()
        Implement timer listener at frequency 1/render_interval to call to render()
        '''
        for robot_id in self.robot_list.keys():
            print("Added listener ","robot_loc_",robot_id)
            rospy.Subscriber('robot_loc_' + str(robot_id), Point, self.update_loc, robot_id)