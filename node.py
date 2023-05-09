class Node:
    def __init__(self):
        self.position = None
        self.parent = None
        self.children = None
        self.num_children = 0
        self.g_cost = None   # exact cost
        self.h_cost = None   # heuristic cost

    @property
    def cost(self):
        return self.g_cost + self.h_cost