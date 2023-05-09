# A* Gridworld Search
import numpy as np
from node import Node
import tools

NUM_ROWS = 10
NUM_COLS = 10
START_POS = [6, 3]
GOAL_POS = [9, 6]

def main():
    # Make a grid and plot it
    data = np.ones((NUM_ROWS, NUM_COLS)) 

    # Make obstacles (setting data to zero)
    data[np.arange(1,7),1] = 0
    data[7,np.arange(1,7)] = 0
    data[np.arange(6,8),7] = 0

    # Define goal and start position
    start_pos = np.array(START_POS)
    goal_pos = np.array(GOAL_POS)
    data[goal_pos[0], goal_pos[1]] = -1
    data[start_pos[0], start_pos[1]] = -2

    # Defining the starting node
    start = Node()
    start.position = start_pos
    start.g_cost = 0
    start.h_cost = tools.get_heuristic_cost(start.position, goal_pos)
    start.num_children = 0

    # Running A* search algorithim and mapping the best path
    data, end_node = tools.run_A_star(start, data, goal_pos)
    data = tools.find_best_path(end_node, data)

    # Plotting the most efficient path from start to finish
    tools.plot_grid(data)

if __name__ == '__main__':
    main()