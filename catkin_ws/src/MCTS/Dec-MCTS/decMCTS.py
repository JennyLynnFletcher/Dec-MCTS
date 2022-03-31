import math
from enum import Enum, auto

import local_sim
import robot
import maze as m

import numpy as np
from collections import defaultdict

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

c_param = 0.5  # Exploration constant, greater than 1/sqrt(8)
discount_param = 0.5


class Agent_State():
    def __init__(self, location, observations):
        self.loc = location
        self.obs = observations


class Agent_Info():
    def __init__(self, state, plan_list, plan_probs, timestamp):
        self.state = state
        self.plan_list = plan_list
        self.plan_probs = plan_probs
        self.time = timestamp

    def select_random_plan(self):
        return np.random.choice(self.plan_list, p=self.plan_probs).copy()


class DecMCTS_Agent(robot.Robot):
    def __init__(self, horizon, *args, **kwargs):
        super(DecMCTS_Agent, self).__init__(*args, **kwargs)
        self.horizon = horizon
        self.other_agent_info = {}

    # TODO: listen for plans from other agents

    def update(self):
        '''
        Move to next position, update observations, update locations, run MCTS,
        publish to ROS topic
        '''
        tree = DecMCTSNode(Agent_State(self.loc, self.observations_list), depth=0)

        # TODO optionally clean up other-agent-plans
        for i in range(10):
            # Perform Dec_MCTS step
            node = tree.select_node(i).expand()
            other_agent_policies = self.sample_other_agents()
            score = node.perform_rollout(self.robot_id, other_agent_policies, self.other_agent_info, self.horizon,
                                         self.get_time(), self.env.get_goal())
            node.backpropagate(score, i)

    def sample_other_agents(self):
        return {i: agent.select_random_plan() for (i, agent) in self.other_agent_info.items()}

    def listener(self):
        '''
        Implement timer listener at frequency 1/time_interval to call to update()
        '''


class DecMCTSNode():
    def __init__(self, state, depth, parent=None, parent_action=None):
        self.depth = depth
        self.state = state
        self.parent = parent
        self.parent_action = parent_action
        self.children = []

        # Incremented during backpropagation
        self.discounted_visits = 0
        self.discounted_score = 0
        self.last_round_visited = 0

    def is_leaf(self):
        return len(self.children) == 0

    def select_node(self, round_n):
        return self.select_node_d_uct(round_n)

    def select_node_d_uct(self, round_n):
        if self.is_leaf():
            return self
        else:
            t_js = [child.discounted_visits * math.pow(discount_param, round_n - child.last_round_visited)
                    for child in self.children]
            t_d = sum(t_js)

            child_scores = []
            for i, child in enumerate(self.children):
                f = child.discounted_score / child.discounted_visits
                c = 2 * math.sqrt(np.log(t_d) / t_js[i])
                child_scores.append(f + c_param * c)

            return self.children[np.argmax(child_scores)]

    # TODO what legal actions exist for unknown positions
    def get_legal_actions(self):
        '''
        Modify according to your game or
        needs. Constructs a list of all
        possible actions from current state.
        Returns a list.
        '''

    def get_stochastic_action(self):
        '''
        Pick a random legal action.
        '''

    def expand(self):
        action = self.get_stochastic_action()
        next_state = self.state.move(action)
        child_node = DecMCTSNode(next_state, self.depth + 1, parent=self, parent_action=action)

        self.children.append(child_node)
        return child_node

    def backpropagate(self, score, iteration):
        self.discounted_visits = 1 + \
                                 math.pow(discount_param, iteration - self.last_round_visited) * self.discounted_visits
        self.discounted_score = score + \
                                np.pow(discount_param, iteration - self.last_round_visited) * self.discounted_score
        self.last_round_visited = iteration
        self.parent.backpropagate(score, iteration)

    def is_fully_expanded(self):
        return len(self.children) >= len(self.get_legal_actions())

    # TODO make sure to mark current location as empty!

    def tree_history(self):
        curr_node = self
        node_actions = []
        start_state = None
        while curr_node.parent is not None:
            node_actions.append(curr_node.parent_action)
            curr_node = curr_node.parent
            if curr_node.parent is None:
                start_state = curr_node.state
        node_actions.reverse()
        return start_state, node_actions

    def perform_rollout(self, this_id, other_agent_policies, other_agent_info, horizon, time, goal):
        maze = m.generate_maze(self.state.obs, goal)
        horizon_time = time + self.depth + horizon

        start_state, node_actions = self.tree_history()

        # Simulate each agent separately
        for id, agent in other_agent_info.items():
            maze.add_robot(id, agent.state.loc)
            maze.simulate_i_steps(horizon_time - agent.time, id, other_agent_policies[id])

        # Score if we took no actions
        maze.add_robot(this_id, start_state.loc)
        null_score = maze.get_score()

        # Score if we take our actual actions
        maze.simulate_i_steps(horizon_time - time, this_id, node_actions)
        actuated_score = maze.get_score()
        return actuated_score - null_score
