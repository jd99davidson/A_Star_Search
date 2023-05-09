import copy
from matplotlib import colors
import matplotlib.pyplot as plt
from node import Node
import numpy as np


def plot_grid(data):
    """ Plot a grid of squares given an array of values. """
    # Create a colormap
    color_map = colors.ListedColormap(['yellow','green','black','blue','red'])
    # Black: obstacles, Blue: open, red: explored, green: goal, yellow: current position
    bounds = [-2, -1, 0, 1, 2, 3]
    norm = colors.BoundaryNorm(bounds, color_map.N)
    fig, ax = plt.subplots()
    ax.imshow(data, cmap=color_map, norm=norm)
    # Draw gridlines
    ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    num_rows, num_cols = data.shape
    ax.set_xticks(np.arange(-0.5, num_cols + 0.1, num_cols / 10))
    ax.set_yticks(np.arange(-0.5, num_rows + 0.1, num_rows / 10))
    plt.show()

def get_heuristic_cost(pos, goal_pos):
    # Heuristic cost (Manhattan Distance)
    return np.abs(pos[0] - goal_pos[0]) + np.abs(pos[1] - goal_pos[1])

def _get_lowest_cost(node):
    """ Find child with lowest cost (recursive through tree). """
    if node.num_children == 0:
        return node, node.cost
    else:
        for i in range(node.num_children):
            if i == 0:
                best_child, lowest_cost = _get_lowest_cost(node.children[i])
            else:
                new_child, new_cost = _get_lowest_cost(node.children[i])
                if new_cost < lowest_cost:
                    lowest_cost = new_cost 
                    best_child = new_child
    return best_child, lowest_cost

def _find_points_to_explore(data, pos):
    """ Look left, right, up, and down for points that are vacant 
        (denoted by red). """

    points = [_get_up_pos(data, pos), _get_down_pos(data, pos),
              _get_left_pos(data, pos), _get_right_pos(data, pos)]
    return [point for point in points if point]

def _is_within_boundaries(data, pos):
    num_rows, num_cols = data.shape
    if (0 <= pos[0] <= num_rows - 1) and (0 <= pos[1] <= num_cols - 1):
        return True
    return False
                                          
def _is_occupiable(data, pos):
    if _is_within_boundaries(data, pos):
        if data[pos[0], pos[1]] == 1 or data[pos[0], pos[1]] == -1:
            return True
    return False

def _get_right_pos(data, pos):
    right_pos = [pos[0], pos[1] + 1]
    if _is_occupiable(data, right_pos):
        return right_pos
    return None

def _get_left_pos(data, pos):
    left_pos = [pos[0], pos[1] - 1]
    if _is_occupiable(data, left_pos):
        return left_pos
    return None

def _get_down_pos(data, pos):
    down_pos = [pos[0] + 1, pos[1]]
    if _is_occupiable(data, down_pos):
        return down_pos
    return None

def _get_up_pos(data, pos):
    up_pos = [pos[0] - 1, pos[1]]
    if _is_occupiable(data, up_pos):
        return up_pos
    return None

def _grow_tree(node, data, goal_pos):
    """ Takes in current node, the map data, and the goal position to """
    explore_points = _find_points_to_explore(data, node.position)
    node.num_children = len(explore_points)
    node.children = []
    for i in range(node.num_children):
        new_child = Node()
        new_child.parent = node
        new_child.position = explore_points[i]

        new_child.g_cost = node.g_cost + 1
        new_child.h_cost = get_heuristic_cost(new_child.position, goal_pos)
        
        node.children.append(new_child)
        # Set the point to explored
        data[new_child.position[0], new_child.position[1]] = 3

    # If there are no possible moves, stop exploring due to high cost
    if node.num_children == 0:
        node.children = None
        node.g_cost = 1000
    return data

def run_A_star(node, data, goal_pos):
    """ Run A* Search Algorithim. """
    goal_found = False
    start_node = node
    while not goal_found:
        data = _grow_tree(node, data, goal_pos)
        node, lowest_cost = _get_lowest_cost(start_node)

        # End when at goal_pos
        if node.position[0] == goal_pos[0] and node.position[1] == goal_pos[1]:
            goal_found = True
    return data, node

def find_best_path(node, data):
    """ Traverse tree to find best path (highlighted in yellow). """
    path_found = False
    while not path_found:
        data[node.position[0], node.position[1]] = -2
        if node.parent == None:
            path_found = True
        else:
            node = node.parent
    return data
