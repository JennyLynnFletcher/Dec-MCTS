from threading import Thread

import environment

if __name__ == '__main__':
    env = environment.Environment(15,15,(6,7), 10, render_interval=1)
    
    for i in range(5):
        env.add_robot()
    
    