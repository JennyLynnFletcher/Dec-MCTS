import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from maze import Action

class Environment():
    def __init__(self, width, height, walls, start, goal, render_interval=0.5):
        self.width = width
        self.height = height
        self.walls = walls
        self.start = start
        self.goal = goal
        self.render_interval = render_interval
        self.timestep = 0
        self.complete = False
        self.robot_list = []  # Tuple of robot ID and location

    def add_robot(self, robot):
        '''
        Add robot with start location start_loc, goal goal_loc and pass self as env
        Add to self.robot_list
        '''

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
        render
        '''

    def update_loc(self, loc_msg):
        '''
        Callback function for robot locations
        Update locations of robots for rendering
        '''

    def listener(self):
        '''
        Implement listener code, call to update_loc()
        Implement timer listener at frequency 1/render_interval to call to render()
        '''
        rospy.init_node('Environment', anonymous=True)
        for (robot_id, _) in self.robot_list:
            rospy.Subscriber('robot_loc_' + robot_id, Point, update_loc, robot_id)
