from enum import Enum, auto

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

class Action(Enum):
    UP = auto()
    RIGHT = auto()
    DOWN = auto()
    LEFT = auto()
    STAY = auto()

def generate_maze(obs, goal):
    return None