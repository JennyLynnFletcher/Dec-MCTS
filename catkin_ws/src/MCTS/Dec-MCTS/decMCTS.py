import math
import random

import local_sim
import robot
import maze as m

import numpy as np
from collections import defaultdict

from scipy import stats

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

c_param = 0.5  # Exploration constant, greater than 1/sqrt(8)
discount_param = 0.5
num_distribution_samples = 50
alpha = 0.01


class Agent_State():
    def __init__(self, location, observations):
        self.loc = location
        self.obs = observations


class Agent_Info():
    def __init__(self, id, state, probs, timestamp):
        self.state = state
        self.probs = probs
        self.time = timestamp
        self.id = id

    def select_random_plan(self):
        plan, _ = np.random.choice(self.probs.keys, p=self.probs.values).copy()
        return plan


# ha ha ha, I'm so sorry if you have to read this part
def uniform_sample_from_all_action_sequences(probs, other_agent_info):
    other_agent_dict_dict = {i: x.probs for (i, x) in other_agent_info.items}
    our_actions, our_node = random.choice(probs.values())
    (other_actions, _), other_qs = zip(*map(lambda x: random.choice(x.items), other_agent_dict_dict.values()))
    our_q = probs[(our_actions, our_node)]
    our_obs = our_node.state.obs
    return our_actions, other_actions, our_q, other_qs, our_obs


class DecMCTS_Agent(robot.Robot):
    def __init__(self, horizon, plan_growth_iterations, *args, **kwargs):
        super(DecMCTS_Agent, self).__init__(*args, **kwargs)
        self.horizon = horizon
        self.other_agent_info = {}
        self.plan_growth_iterations = plan_growth_iterations
        self.executed_action_last_update = True
        self.tree = None
        self.Xrn = []

    # TODO: listen for plans from other agents

    def growSearchTree(self, iterations):

        for i in range(iterations):
            # Perform Dec_MCTS step
            node = self.tree.select_node(i).expand()
            other_agent_policies = self.sample_other_agents()
            score = node.perform_rollout(self.robot_id, other_agent_policies, self.other_agent_info, self.horizon,
                                         self.get_time(), self.env.get_goal())
            node.backpropagate(score, i)

    def reset_tree(self):
        self.Xrn = []
        self.tree = DecMCTSNode(Agent_State(self.loc, self.observations_list), depth=0, Xrn=self.Xrn)

    def get_Xrn_probs(self):
        probs = {}
        for x in self.Xrn:
            probs[x] = 1 / len(self.Xrn)
        return probs

    # Execute_movement should be true if this is a move step, rather than just part of the computation
    def update(self, execute_action=True):
        '''
        Move to next position, update observations, update locations, run MCTS,
        publish to ROS topic
        '''

        if self.executed_action_last_update:
            self.reset_tree()
            self.beta = 0.1
            self.executed_action_last_update = False

        probs = self.get_Xrn_probs()

        t_n = 10  # TODO

        for _ in range(t_n):
            self.growSearchTree(self.plan_growth_iterations)
            self.update_distribution(probs)
            # TODO communication transmit/receive here
            # TODO optionally clean up other-agent-plans
            # To communicate: an agent_info object with our info and probs
            # When receive: override or add the new agent_info object according to the id
            self.cool_beta()

        if execute_action:
            self.executed_action_last_update = True
            best_plan,_ = max(probs, key=probs.get)
            #TODO actualy execute plan

    def cool_beta(self):
        self.beta = self.beta*0.9

    def sample_other_agents(self):
        return {i: agent.select_random_plan() for (i, agent) in self.other_agent_info.items()}

    def listener(self):
        '''
        Implement timer listener at frequency 1/time_interval to call to update()
        '''

    def update_distribution(self, probs):
        # TODO sample

        for (x, node) in probs.keys:
            q = probs[(x, node)]
            e_f = 0
            e_f_x = 0
            for i in range(num_distribution_samples):
                # TODO Do we generate a maze based off of our simulated observations or only what we actually know?
                our_actions, other_actions, our_q, other_qs, our_obs = \
                    uniform_sample_from_all_action_sequences(probs, self.other_agent_info)
                f = compute_f(self.robot_id, our_actions, other_actions, self.loc, our_obs,
                              self.other_agent_info, self.horizon, self.get_time(), self.env.get_goal())

                f_x = compute_f(self.robot_id, x, other_actions, self.loc, node.state.obs,
                                self.other_agent_info, self.horizon, self.get_time(), self.env.get_goal())

                e_f += np.prod(other_qs + [our_q]) * f
                e_f_x += np.prod(other_qs) * f_x

            probs[(x, node)] = q - alpha * q * (
                    (e_f - e_f_x) / self.beta
                    + stats.entropy(probs.values) + np.log(q))

            # normalize
            factor = 1.0 / sum(probs.values())
            for k in probs:
                probs[k] = probs[k] * factor


class DecMCTSNode():
    def __init__(self, state, depth, Xrn, parent=None, parent_action=None):
        self.depth = depth
        self.state = state
        self.parent = parent
        self.parent_action = parent_action
        self.children = []
        self.Xrn = Xrn

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

        horizon_time = time + self.depth + horizon

        start_state, node_actions = self.tree_history()

        self.Xrn.append((node_actions, self))

        return compute_f(this_id, node_actions, other_agent_policies, start_state.loc, self.state.obs,
                         other_agent_info, horizon_time, time, goal)


def compute_f(our_id, our_policy, other_agent_policies, our_loc, our_obs, other_agent_info, steps, current_time, goal):
    maze = m.generate_maze(our_obs, goal)
    # Simulate each agent separately (simulates both history and future plans)
    for id, agent in other_agent_info.items():
        maze.add_robot(id, agent.state.loc)
        maze.simulate_i_steps(steps - agent.time, id, other_agent_policies[id])

    # Score if we took no actions
    maze.add_robot(our_id, our_loc)
    null_score = maze.get_score()

    # Score if we take our actual actions (simulates future plans)
    maze.simulate_i_steps(steps - current_time, our_id, our_policy)
    actuated_score = maze.get_score()
    return actuated_score - null_score
