from enum import Enum, auto
import collections
import random

import maze_gen


class Maze:
    def __init__(self, goal, maze_walls):
        self.goal = goal
        self.agent_positions = {}
        self.walls = maze_walls

    def simulate_i_steps(self, steps, agent_id, policy):
        valid_policy = True  # If policy conflicts with facts, end at last valid position and default
        for i in range(steps):
            if i < len(policy) and valid_policy:
                if not self.try_action(agent_id, policy[i]):
                    valid_policy = False

            # Inverse so that changing policy to invalid immediately jumps here
            if not (i < len(policy) and valid_policy):
                while not self.execute_default_action(agent_id):
                    pass

    def try_action(self, agent_id, action):
        x, y = self.agent_positions[agent_id]
        new_position = [x, y]
        if action == Action.UP:
            new_position[1] -= 1
        elif action == Action.DOWN:
            new_position[1] += 1
        elif action == Action.RIGHT:
            new_position[0] += 1
        elif action == Action.LEFT:
            new_position[0] -= 1
        if self.walls[new_position[1], new_position[0]]:
            self.agent_positions[agent_id] = tuple(new_position)
            return True
        else:
            return False

    def execute_default_action(self, agent_id):
        action = random.choice(list(Action))
        return self.try_action(agent_id, action)

    def get_score(self, agent_obs, comms_aware=False):
        # print(sparse.lil_matrix(agent_obs).toarray())
        # t = time()
        w = self.walls.shape[1]
        h = self.walls.shape[0]
        positions_all = list(self.agent_positions.values())
        n_xplored = (agent_obs == 1).getnnz()

        total_explorable_area = ((h - 2) * (w - 2) + h + w - 5) / 2
        percent_explored = n_xplored / total_explorable_area

        # newt = time()
        # print(newt - t, "init")
        # t = time()

        distance = {self.goal: 0}
        visited = set()
        visited.add(self.goal)
        goal_connected = {self.goal}
        bfs_queue = collections.deque()
        bfs_queue.append(self.goal)
        max_distance = 0

        # newt = time()
        # print(newt - t, "bfs_init")
        # t = time()
        for x, y in positions_all:
            self.walls[y,x] = 1

        while bfs_queue:
            current = bfs_queue.pop()
            y, x = current
            for dif in (-1, 1):
                for neighbour in [(y + dif, x), (y, x + dif)]:
                    if neighbour in visited or (not self.walls[neighbour]):
                        continue                    
                    visited.add(neighbour)
                    distance[neighbour] = distance[current] + 1
                    max_distance = max(distance[neighbour], max_distance)
                    if (y,x) in goal_connected and not agent_obs[neighbour]:
                        goal_connected.add(neighbour)
                    bfs_queue.append(neighbour)

        # newt = time()
        # print(newt - t, "bfs")
        # t = time()

        goal_component_size_pct = len(goal_connected) / total_explorable_area
            
        
        robot_distances = [distance[(position[1], position[0])] if (position[1], position[0]) in distance.keys() else max_distance for position in positions_all]
            

            
        # TODO: think about the weights on these things, for now they are all equal
        #  which is a really bad idea since percent explored < 1 and others are >> 1
        score = - sum(robot_distances)/(len(robot_distances) * max_distance) + 10*percent_explored - goal_component_size_pct

        if comms_aware:
            max_dist = 0
            sum_dists = 0
            n_dists = 0
            for i in range(len(positions_all)):
                for j in range(i):
                    x1, y1 = positions_all[i]
                    x2, y2 = positions_all[j]
                    dist = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** (1 / 2)

                    max_dist = max(max_dist, dist)
                    n_dists += 1
                    sum_dists += dist

            score -= max_dist / max_distance

        # newt = time()
        # print(newt - t, "score")

        return score

    # loc is (x,y) tuple
    def add_robot(self, robot_id, loc):
        self.agent_positions[robot_id] = loc


class Action(Enum):
    UP = auto()
    RIGHT = auto()
    DOWN = auto()
    LEFT = auto()
    STAY = auto()


def generate_maze(obs, goal):
    maze_walls = maze_gen.fill_in_maze(obs.copy())
    maze_walls = maze_gen.fill_in_maze(maze_walls.maximum(obs.copy()))

    return Maze((goal[1],goal[0]), maze_walls)
