import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point


class Robot(object):
    def __init__(self, robot_id, start_loc, goal_loc, time_interval, env):
        self.robot_id = robot_id
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.loc = start_loc
        self.loc_log = [start_loc]
        self.observations_list = []
        self.env = env
        self.pub_loc = rospy.Publisher('robot_loc_' + robot_id, Point, queue_size=10)
        rospy.init_node('Agent', anonymous=True)
        self.rate = rospy.Rate(5)

    def get_observations(self):
        '''
        Return a dictionary of N,E,S,W and whether there
        is a wall in that direction from the robot's current
        location
        '''
        return self.env.get_walls_from_loc(self.loc)

    def move(self, direction):
        '''
        Move one step in direction passed as argument
        if fail due to obstructing wall from get_observations()
        stay in same location, update log
        '''
        if direction == N and not self.observations_list[-1]['N']:
            (x,y) = self.loc
            self.loc = (x, y+1)
        elif direction == E and not self.observations_list[-1]['E']:
            (x,y) = self.loc
            self.loc = (x+1, y)
        elif direction == S and not self.observations_list[-1]['S']:
            (x,y) = self.loc
            self.loc = (x, y-1)
        elif direction == W and not self.observations_list[-1]['W']:
            (x,y) = self.loc
            self.loc = (x-1, y)
            
        self.observations_list.append(self.loc)

    def update(self,execute_action=True):
        '''
        Move to next position, update observations, update locations, run MCTS,
        publish to ROS topic
        '''
        msg = Point(x=self.loc[0], y=self.loc[1])
        self.pub_loc.publish(msg)
   

    def listener(self):
        '''
        Implement timer listener at frequency 1/time_interval to call to update()
        '''
