import math
import os
import random
import threading
import time

import maze as m

import numpy as np

from scipy import stats
from maze import Action
from scipy import sparse

import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point
import pickle
import codecs

c_param = 0.5  # Exploration constant, greater than 1/sqrt(8)
discount_param = 0.90
alpha = 0.01


def printsparse(matrix):
    print(matrix.toarray())


class Agent_State():
    def __init__(self, location, observations):
        self.loc = location
        self.obs = observations


def move(loc, action):
    x, y = loc
    if action == Action.UP:
        loc = (x, y - 1)
    elif action == Action.DOWN:
        loc = (x, y + 1)
    elif action == Action.LEFT:
        loc = (x - 1, y)
    elif action == Action.RIGHT:
        loc = (x + 1, y)
    else:
        loc = (x, y)
    return loc


def update_agent_state(state, action):
    loc = move(state.loc, action)
    x, y = loc
    obs = state.obs.copy()
    obs[y, x] = 1
    return Agent_State(loc, obs)


class Agent_Info():
    def __init__(self, robot_id, state, probs, time, complete):
        self.state = state  # Agent_State
        self.probs = probs  # dict
        self.time = time  # int
        self.robot_id = robot_id  # arbitrary
        self.complete = complete

    def select_random_plan(self):
        return np.random.choice(list(self.probs.keys()), p=list(self.probs.values())).get_action_sequence()


# ha ha ha, I'm so sorry if you have to read this part
def uniform_sample_from_all_action_sequences(probs, other_agent_info):
    other_actions = {}
    other_qs = {}
    for i, x in other_agent_info.items():
        chosen_node = random.choice(list(x.probs.keys()))
        chosen_q = x.probs[chosen_node]
        chosen_actions = chosen_node.get_action_sequence()
        other_actions[i] = chosen_actions
        other_qs[i] = chosen_q

    our_node = random.choice(list(probs.keys()))
    our_q = probs[our_node]
    our_obs = our_node.state.obs
    our_actions = our_node.get_action_sequence()
    return our_actions, other_actions, our_q, other_qs, our_obs


def get_new_prob(x):
    node, probs, distribution_sample_iterations, other_agent_info, determinization_iterations, robot_id, \
    observations_list, loc, horizon, time, goal, comms_aware_planning, beta, complete = x
    q = probs[node]
    e_f = 0
    e_f_x = 0
    for _ in range(distribution_sample_iterations):
        # Evaluate nodes based off of what we actually know
        our_actions, other_actions, our_q, other_qs, our_obs = \
            uniform_sample_from_all_action_sequences(probs, other_agent_info)
        f = 0
        f_x = 0
        for _ in range(determinization_iterations // 2):
            f += compute_f(robot_id, our_actions, other_actions, observations_list, loc, our_obs,
                           other_agent_info, horizon, time, goal,
                           comms_aware_planning=comms_aware_planning, complete=complete) \
                 / (determinization_iterations // 2)

            f_x += compute_f(robot_id, node.get_action_sequence(), other_actions, observations_list,
                             loc, our_obs,
                             other_agent_info, horizon, time, goal,
                             comms_aware_planning=comms_aware_planning, complete=complete) \
                   / (determinization_iterations // 2)

        e_f += np.prod(list(other_qs.values()) + [our_q]) * f
        if len(other_qs) > 0:
            e_f_x += np.prod(list(other_qs.values())) * f_x
        else:
            e_f_x = 0
        return node, q - alpha * q * (
                (e_f - e_f_x) / beta
                + stats.entropy(list(probs.values())) + np.log(q))


class DecMCTS_Agent():
    # Runtime is
    # prob_update_iterations * probs_size * determinationization_iterations * distribution_sample_iterations
    def __init__(self, robot_id, start_loc, goal_loc, env,
                 horizon=10,
                 prob_update_iterations=5,
                 plan_growth_iterations=30,
                 distribution_sample_iterations=5,
                 determinization_iterations=3,
                 probs_size=12,
                 out_of_date_timeout=None,
                 comms_drop=None,
                 comms_drop_rate=None,
                 comms_aware_planning=False,
                 name="default"):
        self.horizon = horizon
        self.other_agent_info = {}
        self.executed_action_last_update = True
        self.tree = None
        self.Xrn = []
        self.reception_queue = []
        self.pub_obs = rospy.Publisher('robot_obs_' + name, String, queue_size=20)
        self.pub_obs = rospy.Publisher('robot_obs_' + name, String, queue_size=10)
        self.update_iterations = 0
        self.prob_update_iterations = prob_update_iterations
        self.plan_growth_iterations = plan_growth_iterations
        self.determinization_iterations = determinization_iterations
        self.distribution_sample_iterations = distribution_sample_iterations
        self.out_of_date_timeout = out_of_date_timeout
        self.time = 0
        self.probs_size = probs_size
        self.comms_aware_planning = comms_aware_planning
        self.times_removed_other_agent = 0

        self.robot_id = robot_id
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.loc = start_loc
        self.loc_log = [start_loc]
        self.env = env
        self.pub_loc = rospy.Publisher('robot_loc_' + str(robot_id) + "_" + name, Point, queue_size=20)
        self.comms_drop = comms_drop
        self.comms_drop_rate = comms_drop_rate
        self.complete = False
        self.missed_messages = 0

        self.observations_list = sparse.dok_matrix((self.env.height, self.env.width))
        self.add_edges_to_observations()
        self.update_observations_from_location()
        msg = Point(x=self.loc[0], y=self.loc[1])
        self.pub_loc.publish(msg)

        print("Creating robot " + str(self.robot_id) + " at position " + str(start_loc) +
              ", comms aware planning is " + ("enabled" if comms_aware_planning else "disabled"))

    def add_edges_to_observations(self):
        for i in range(self.env.width):
            self.observations_list[0, i] = 2
            self.observations_list[self.env.height - 1, i] = 2

        for i in range(self.env.height):
            self.observations_list[i, 0] = 2
            self.observations_list[i, self.env.width - 1] = 2

    def get_time(self):
        return self.time

    def growSearchTree(self):

        for i in range(self.plan_growth_iterations):
            # Perform Dec_MCTS step
            node = self.tree.select_node(i).expand()
            other_agent_policies = self.sample_other_agents()
            score = node.perform_rollout(self.robot_id, other_agent_policies, self.other_agent_info,
                                         self.observations_list, self.horizon,
                                         self.get_time(), self.determinization_iterations, self.env.get_goal(),
                                         self.complete)
            node.backpropagate(score, i)

    def reset_tree(self):
        self.Xrn = []
        self.tree = DecMCTSNode(Agent_State(self.loc, self.observations_list), depth=0,
                                maze_dims=(self.env.height, self.env.width), Xrn=self.Xrn,
                                comms_aware_planning=self.comms_aware_planning)

    def get_Xrn_probs(self):
        self.Xrn.sort(reverse=True, key=(lambda node: node.discounted_score))
        probs = {}
        for node in self.Xrn[1:min(len(self.Xrn), self.probs_size)]:
            probs[node] = 1 / self.probs_size
        return probs

    def package_comms(self, probs):
        return Agent_Info(self.robot_id, Agent_State(self.loc, self.observations_list), probs, self.get_time()
                          , self.complete)

    def unpack_comms(self):
        for message_str in self.reception_queue:
            message = pickle.loads(codecs.decode(message_str.data.encode(), 'base64'))
            if message.robot_id != self.robot_id:
                distance = max(
                    math.sqrt((message.state.loc[0] - self.loc[0]) ** 2 + (message.state.loc[1] - self.loc[1]) ** 2), 1)
                if self.comms_drop == "uniform" and random.random() < self.comms_drop_rate:
                    self.missed_messages += 1
                    # print("Packet drop")
                elif self.comms_drop == "distance" and random.random() < self.comms_drop_rate / (distance ** 2):
                    self.missed_messages += 1
                    # print("Packet drop")
                else:
                    robot_id = message.robot_id
                    # If seen before
                    if robot_id in self.other_agent_info.keys():
                        # If fresh message
                        if message.time >= self.other_agent_info[robot_id].time:
                            self.other_agent_info[robot_id] = message
                    else:
                        self.other_agent_info[robot_id] = message
                    self.observations_list = merge_observations(self.observations_list, message.state.obs)
        time = self.get_time()
        # Filter out-of-date messages
        if self.out_of_date_timeout is not None:
            new_other_agent = {}
            for k, v in self.other_agent_info.items():
                if v.time + self.out_of_date_timeout >= time:
                    new_other_agent[k] = v
                else:
                    self.times_removed_other_agent += 1
            self.other_agent_info = new_other_agent

        self.reception_queue = []

    # Execute_movement should be true if this is a move step, rather than just part of the computation
    def update(self, execute_action=True):
        '''
        Move to next position, update observations, update locations, run MCTS,
        publish to ROS topic
        '''
        print("-------- Update ", self.update_iterations, " Robot id ", self.robot_id, " Execute action ",
              execute_action, "Thread id ", threading.current_thread().name, "---------")
        self.update_iterations += 1
        if self.executed_action_last_update:
            self.reset_tree()
            self.beta = 0.1
            self.executed_action_last_update = False

        probs = self.get_Xrn_probs()

        for i in range(self.prob_update_iterations):
            self.growSearchTree()
            if len(probs.keys()) > 0:
                probs = self.update_distribution(probs)
                message = self.package_comms(probs)
                self.pub_obs.publish(codecs.encode(pickle.dumps(message), "base64").decode())

            self.unpack_comms()
            self.cool_beta()

        if execute_action:
            self.executed_action_last_update = True
            if len(probs) > 0:
                best_plan = max(probs, key=probs.get).get_action_sequence()
                best_action = best_plan[0]
            else:
                best_action = Action.STAY
            print("Executing action: " + str(best_action))
            self.loc = move(self.loc, best_action)
            if self.loc == self.goal_loc:
                self.complete = True
            msg = Point(x=self.loc[0], y=self.loc[1])
            self.pub_loc.publish(msg)
            self.update_observations_from_location()

    def update_observations_from_location(self):
        x, y = self.loc
        walls = self.env.get_walls_from_loc(self.loc)
        if walls[Action.UP]:
            self.observations_list[y - 1, x] = 2
        else:
            self.observations_list[y - 1, x] = 1
        if walls[Action.DOWN]:
            self.observations_list[y + 1, x] = 2
        else:
            self.observations_list[y + 1, x] = 1
        if walls[Action.LEFT]:
            self.observations_list[y, x - 1] = 2
        else:
            self.observations_list[y, x - 1] = 1
        if walls[Action.RIGHT]:
            self.observations_list[y, x + 1] = 2
        else:
            self.observations_list[y, x + 1] = 1
        self.observations_list[y, x] = 1

    def cool_beta(self):
        self.beta = self.beta * 0.9

    def sample_other_agents(self):
        return {i: agent.select_random_plan() for (i, agent) in self.other_agent_info.items()}

    def update_distribution(self, probs):
        args = [(node, probs, self.distribution_sample_iterations, self.other_agent_info,
                 self.determinization_iterations,
                 self.robot_id, self.observations_list, self.loc, self.horizon, self.get_time(),
                 self.env.get_goal(), self.comms_aware_planning, self.beta, self.complete) for node in
                list(probs.keys())]

        newprobs = list(map(get_new_prob, args))
        probs = {}

        # normalize
        factor = sum([prob for node, prob in newprobs])
        for node, prob in newprobs:
            probs[node] = prob / factor
        return probs


class DecMCTSNode():
    def __init__(self, state, depth, maze_dims, Xrn, parent=None, parent_action=None, comms_aware_planning=False):
        self.depth = depth
        self.state = state
        self.parent = parent
        self.parent_action = parent_action
        self.children = []
        self.Xrn = Xrn
        self.maze_dims = maze_dims
        self.comms_aware_planning = comms_aware_planning

        # Incremented during backpropagation
        self.discounted_visits = 0
        self.discounted_score = 0
        self.last_round_visited = 0
        self.unexplored_actions = self.get_legal_actions()
        self.action_sequence = None
        self.comms_aware_planning = comms_aware_planning

    def get_action_sequence(self):
        if self.action_sequence is not None:
            return self.action_sequence
        else:
            _, self.action_sequence = self.tree_history()
            return self.action_sequence

    def select_node(self, round_n):
        return self.select_node_d_uct(round_n)

    def select_node_d_uct(self, round_n):
        if self.not_fully_explored():
            return self
        else:
            t_js = [child.discounted_visits * math.pow(discount_param, round_n - child.last_round_visited)
                    for child in self.children]
            t_d = sum(t_js)

            child_scores = []
            for i, child in enumerate(self.children):
                if child.discounted_visits == 0:
                    print("WARNING, CHILDREN SHOULD NOT BE UNVISITED (decmcts.py  select_node_d_uct)")
                    print(child.discounted_visits)
                    print(self.depth)
                    print(self.discounted_visits)
                    f = child.discounted_score
                    c = 2 * math.sqrt(max(np.log(t_d), 0))
                else:
                    f = child.discounted_score / child.discounted_visits
                    c = 2 * math.sqrt(max(np.log(t_d), 0) / t_js[i])
                child_scores.append(f + c_param * c)

            return self.children[np.argmax(np.asarray(child_scores))].select_node_d_uct(round_n)

    def get_legal_actions(self):
        '''
        Modify according to your game or
        needs. Constructs a list of all
        possible actions from current state.
        Returns a list.
        '''
        x, y = self.state.loc
        actions = []
        if y - 1 >= 0 and self.state.obs[y - 1, x] != 2:
            actions.append(Action.UP)
        if y + 1 < self.maze_dims[0] and self.state.obs[y + 1, x] != 2:
            actions.append(Action.DOWN)
        if x - 1 >= 0 and self.state.obs[y, x - 1] != 2:
            actions.append(Action.LEFT)
        if x + 1 < self.maze_dims[1] and self.state.obs[y, x + 1] != 2:
            actions.append(Action.RIGHT)
        return actions

    def get_stochastic_action(self):
        '''
        Pick a random legal action.
        '''
        return random.choice(self.unexplored_actions)

    def expand(self):
        action = self.get_stochastic_action()
        self.unexplored_actions.remove(action)
        next_state = update_agent_state(self.state, action)
        child_node = DecMCTSNode(next_state, self.depth + 1, self.maze_dims, self.Xrn, parent=self,
                                 parent_action=action, comms_aware_planning=self.comms_aware_planning)

        self.children.append(child_node)
        return child_node

    def backpropagate(self, score, iteration):
        self.discounted_visits = 1 + \
                                 math.pow(discount_param, iteration - self.last_round_visited) * self.discounted_visits
        self.discounted_score = score + \
                                math.pow(discount_param, iteration - self.last_round_visited) * self.discounted_score
        self.last_round_visited = iteration
        if self.parent is not None:
            self.parent.backpropagate(score, iteration)

    def not_fully_explored(self):
        return len(self.unexplored_actions) != 0

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

    def perform_rollout(self, this_id, other_agent_policies, other_agent_info, real_obs, horizon, time,
                        determinization_iterations, goal, complete):

        horizon_time = time + self.depth + horizon
        start_state, node_actions = self.tree_history()
        self.Xrn.append(self)

        avg = 0
        for _ in range(determinization_iterations):
            avg += compute_f(this_id, node_actions, other_agent_policies, real_obs, start_state.loc, self.state.obs,
                             other_agent_info, horizon_time, time, goal,
                             comms_aware_planning=self.comms_aware_planning,
                             complete=complete) / determinization_iterations
        return avg


def compute_f(our_id, our_policy, other_agent_policies, real_obs, our_loc, our_obs, other_agent_info, steps,
              current_time, goal, comms_aware_planning, complete):
    maze = m.generate_maze(our_obs, goal)
    # Simulate each agent separately (simulates both history and future plans)
    for id, agent in other_agent_info.items():
        maze.add_robot(id, agent.state.loc, agent.state.complete)
        maze.simulate_i_steps(steps - agent.time, id, other_agent_policies[id])

    # Score if we took no actions
    maze.add_robot(our_id, our_loc, complete)
    null_score = maze.get_score(real_obs, comms_aware_planning=comms_aware_planning)

    # Score if we take our actual actions (simulates future plans)
    maze.simulate_i_steps(steps - current_time, our_id, our_policy)
    actuated_score = maze.get_score(real_obs, comms_aware_planning=comms_aware_planning)
    return actuated_score - null_score


def merge_observations(obs1, obs2):
    return obs1.maximum(obs2)
