class Maze():
    def simulate_i_steps(self,i, agent_id, policy):
        valid_policy = True  # If policy conflicts with facts, end at last valid position and default
        if len(policy) > i and valid_policy:
            if self.valid_move(agent_id, policy[i]):
                self.execute_move(agent_id, policy[i])
            else:
                valid_policy = False

            # Inverse so that changing policy to invalid immediately jumps here
        if not (len(policy) > i and valid_policy):
            self.excute_default_move(agent_id)

    def valid_move(self, agent_id, param):
        pass

    def execute_move(self, agent_id, param):
        pass

    def excute_default_move(self, agent_id):
        pass

class Action(Enum):
    UP = auto()
    RIGHT = auto()
    DOWN = auto()
    LEFT = auto()
    STAY = auto()

def generate_maze(obs, goal):
    return None