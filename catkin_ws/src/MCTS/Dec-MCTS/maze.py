from enum import Enum, auto
import collections

from scipy import sparse

import maze_gen


class Maze():
    def simulate_i_steps(self,steps, agent_id, policy):
        valid_policy = True  # If policy conflicts with facts, end at last valid position and default
        for i in range(steps):
            if i < len(policy) and valid_policy:
                if self.valid_action(agent_id, policy[i]):
                    self.execute_action(agent_id, policy[i])
                else:
                    valid_policy = False

            # Inverse so that changing policy to invalid immediately jumps here
            if not (i < len(policy) and valid_policy):
                self.execute_default_action(agent_id)

    def valid_action(self, agent_id, action):
        pass

    def execute_action(self, agent_id, action):
        pass

    def execute_default_action(self, agent_id):
        pass

    # loc is (x,y) tuple
    def add_robot(self, id, loc):
        pass

    def get_score(self, comms_aware=False):
        # TODO: replace code starting here with proper code once stuff is decided
        # ----------------- FILLER CODE START -----------------
        enviornment = None
        env = enviornment.Environment(7, 7, maze_gen.generate_maze(5, 5), (1, 1), (5, 5))
        agent_obs = {5: sparse.dok_matrix((7, 7)), 1: sparse.dok_matrix((7, 7))}  # id to dok matrix obs should work
        robot_locs = [(3, 3), (1, 1)]
        # ------------------ FILLER CODE END ------------------
        goal = env.goal
        w = env.width
        h = env.height
        # TODO: figure out how to deal with height and width of maze matrix being + 2 sized, this may be dumb
        explored = sparse.dok_matrix(sum(agent_obs.values(), start=sparse.dok_matrix((h, w))), dtype=bool)
        total_explorable_area = ((h - 2) * (w - 2) + h + w - 5) / 2
        percent_explored = len(explored) / total_explorable_area

        distance = sparse.dok_matrix((h, w), dtype=int)
        visited = sparse.dok_matrix((h, w), dtype=bool)
        visited[goal] = True
        goal_connected = sparse.dok_matrix((h, w), dtype=bool)
        goal_connected[goal] = True
        bfs_queue = collections.deque()
        bfs_queue.append(goal)
        while bfs_queue:
            current = bfs_queue.pop()
            y, x = current
            for dif in (-1, 1):
                for neighbour in [(y + dif, x), (y, x + dif)]:
                    if visited[neighbour] or (not env.walls[neighbour]):
                        continue
                    visited[neighbour] = True
                    distance[neighbour] = distance[current] + 1
                    goal_connected[neighbour] = goal_connected[current] and (not explored[neighbour])
                    bfs_queue.append(neighbour)

        goal_component_size = len(goal_connected)
        robot_distances = [distance.T[robot_loc] for robot_loc in robot_locs]

        # TODO: think about the weights on these things, for now they are all equal
        #  which is a really bad idea since percent explored < 1 and others are >> 1
        score = sum(robot_distances) + percent_explored + goal_component_size

        if comms_aware:
            max_dist = 0
            sum_dists = 0
            n_dists = 0
            for i in range(len(robot_locs)):
                for j in range(i):
                    x1, y1 = robot_locs[i]
                    x2, y2 = robot_locs[j]
                    dist = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** (1 / 2)

                    max_dist = max(max_dist, dist)
                    n_dists += 1
                    sum_dists += dist

            score += max_dist
        return score


class Action(Enum):
    UP = auto()
    RIGHT = auto()
    DOWN = auto()
    LEFT = auto()
    STAY = auto()


def generate_maze(obs, goal):
    return None