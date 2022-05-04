
class Edge:
    def __init__(self, x, y, gr_node, direction, type, obstacles = []):
        self.x = x
        self.y = y
        self.gr_node = gr_node
        self.direction = direction
        self.type = type
        self.obstacles = obstacles

class PathPlanner:
    def __init__(self, nodes_list=None, tasks_list=None):

        self.nodes_list = nodes_list

        if nodes_list is None:
            self.construct_nodes_list(tasks_list)

    def construct_nodes_list(self, tasks_list):

        edges_of_interest = []
        for task in tasks_list:
