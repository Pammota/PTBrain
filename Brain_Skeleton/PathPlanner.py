import copy
import collections

import cv2
import numpy as np

DIR_UP = 0
DIR_RIGHT = 1
DIR_DOWN = 2
DIR_LEFT = 3


class Edge:
    def __init__(self, direction, type, tasks=[], xy=None):
        self.xy = xy
        self.x = None
        self.y = None
        self.direction = direction
        self.type = type
        self.tasks = tasks


class Node:
    def __init__(self, gr_node, coords, neighbors):
        self.gr_node = gr_node
        self.coords = coords
        self.neighbors = neighbors
        self.prev = None
        self.final = False
        self.passed = False


def compute_direction(edge, next_edge):
    direction = ""
    edgedir = edge.direction
    next_edgedir = next_edge.direction

    if next_edge.type == "roundabout":
        direction = "roundabout_"
        if edge.type == "highway":
            direction = "highway_roundabout_"
            edgedir = DIR_UP
        elif edge.type == "normal" and edge.xy == "GJ":
            edgedir = DIR_LEFT
        elif edge.type == "normal" and edge.xy == "IJ":
            edgedir = DIR_RIGHT
        elif edge.type == "normal" and edge.xy == "CK":
            edgedir = DIR_LEFT
    elif next_edge.type == "country":
        direction = "country_"
    elif next_edge.type == "highway":
        direction = "highway_"
    elif next_edge.type == "highway_exit":
        direction = "highway_exit_"
        edgedir = DIR_LEFT
    elif next_edge.type == "normal":
        if edge.type == "roundabout":
            edgedir = DIR_DOWN
        elif edge.type == "country":
            edgedir = DIR_UP
        elif edge.type == "normal" and edge.xy == "KG":
            edgedir = DIR_DOWN

    if edgedir == next_edgedir:
        direction += "forward"
    if (edgedir - next_edgedir) % 4 == 1:
        direction += "left"
    if (edgedir - next_edgedir) % 4 == 3:
        direction += "right"
    # == 2 should not be possible

    return direction


class PathPlanner:
    def __init__(self, nodes_list=None, tasks_list=None):

        self.image = cv2.imread("track.png")

        self.edges = {}
        self.nodes = {}
        self.__declare_edges()

        self.nodes_list = nodes_list
        self.num_sections = 0

        if nodes_list is None:
            self.construct_nodes_list(tasks_list)

        self.node_idx = 0

    def __declare_edges(self):
        ###### insert the edges
        self.edges["0A"] = Edge(DIR_UP, "start", ["start"])

        self.edges["A0"] = Edge(DIR_DOWN, "stop", ["stop"])
        self.edges["AB"] = Edge(DIR_RIGHT, "normal", [])
        self.edges["AD"] = Edge(DIR_UP, "normal", ["crosswalk"])

        self.edges["BA"] = Edge(DIR_LEFT, "normal", [])
        self.edges["BC"] = Edge(DIR_RIGHT, "normal", [])
        self.edges["BE"] = Edge(DIR_UP, "normal", ["semaphore"])

        self.edges["CB"] = Edge(DIR_LEFT, "highway_exit", [])
        self.edges["CK"] = Edge(DIR_RIGHT, "country", ["tailing"], "CK")
        self.edges["CJ"] = Edge(DIR_RIGHT, "highway", ["highway", "roundabout"])
        self.edges["CF"] = Edge(DIR_UP, "highway_exit", [])

        self.edges["DA"] = Edge(DIR_DOWN, "normal", ["crosswalk"])
        self.edges["DE"] = Edge(DIR_RIGHT, "normal", ["semaphore"])
        self.edges["DG"] = Edge(DIR_UP, "normal", [])

        self.edges["EB"] = Edge(DIR_DOWN, "normal", [])
        self.edges["ED"] = Edge(DIR_LEFT, "normal", [])
        self.edges["EF"] = Edge(DIR_RIGHT, "normal", [])
        self.edges["EH"] = Edge(DIR_UP, "normal", ["one_way", "closed_road", "tailing"])

        self.edges["FC"] = Edge(DIR_DOWN, "normal", [])
        self.edges["FE"] = Edge(DIR_LEFT, "normal", ["semaphore"])
        self.edges["FI"] = Edge(DIR_UP, "normal", [])

        self.edges["GD"] = Edge(DIR_DOWN, "normal", [])
        self.edges["GH"] = Edge(DIR_RIGHT, "normal", [])
        self.edges["GJ"] = Edge(DIR_UP, "normal", ["ramp", "crosswalk", "roundabout"], "GJ")

        self.edges["HG"] = Edge(DIR_LEFT, "normal", ["pedestrian"])
        self.edges["HI"] = Edge(DIR_RIGHT, "normal", [])

        self.edges["IF"] = Edge(DIR_DOWN, "normal", [])
        self.edges["IH"] = Edge(DIR_LEFT, "normal", [])
        self.edges["IJ"] = Edge(DIR_UP, "normal", ["roundabout", "crosswalk"], "IJ")

        self.edges["JC"] = Edge(DIR_DOWN, "roundabout", ["highway"])
        self.edges["JG"] = Edge(DIR_RIGHT, "roundabout", ["ramp", "crosswalk", "parking"])
        self.edges["JI"] = Edge(DIR_LEFT, "roundabout", ["crosswalk"])

        self.edges["KG"] = Edge(DIR_RIGHT, "normal", ["ramp", "crosswalk", "parking"], "KG")
        self.edges["KJ"] = Edge(DIR_LEFT, "normal", ["roundabout"])

        ######## insert the nodes
        self.nodes["0"] = Node(None, (53, 649), ["A"])
        self.nodes["A"] = Node(None, (53, 582), ["0", "D", "B"])
        self.nodes["B"] = Node(None, (180, 582), ["A", "E", "C"])
        self.nodes["C"] = Node(None, (302, 582), ["B", "F", "J", "K"])
        self.nodes["D"] = Node(None, (53, 462), ["A", "G", "E"])
        self.nodes["E"] = Node(None, (180, 462), ["B", "D", "H", "F"])
        self.nodes["F"] = Node(None, (302, 462), ["C", "I", "E"])
        self.nodes["G"] = Node(None, (53, 301), ["D", "J", "H"])
        self.nodes["H"] = Node(None, (180, 301), ["G", "I"])
        self.nodes["I"] = Node(None, (302, 301), ["F", "H", "J"])
        self.nodes["J"] = Node(None, (585, 172), ["C", "I", "G"])
        self.nodes["K"] = Node(None, (702, 172), ["J", "G"])

        ###### draw all nodes on graph
        for id, properties in self.nodes.items():
            cv2.putText(self.image, id, properties.coords, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        ##### draw all edges
        for edge, descr in self.edges.items():
            x = edge[0]
            y = edge[1]
            cv2.arrowedLine(self.image, self.nodes[x].coords, self.nodes[y].coords, (0, 128, 128), 1)

        """cv2.imshow("graph", self.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()"""

    def shortest_path_unweighted(self, starting_point, graph):

        queue = [starting_point]
        dest = None

        graph["nodes"][starting_point].prev = "-"

        while (len(queue) > 0):
            try:
                crt_node = queue.pop(0)
                for neighb in graph["nodes"][crt_node].neighbors:
                    if graph["nodes"][neighb].passed is False and \
                            graph["nodes"][crt_node].prev[0] != neighb[0]:
                        if graph["nodes"][crt_node].prev[0] == "K" and neighb[0] == "G":
                            # problem here!!! If we come in J from K the first time and then from I
                            # Even if we came from I it would be discarded
                            continue
                        if graph["nodes"][crt_node].prev[0] == "J" and neighb[0] == "K":
                            continue
                        if graph["nodes"][crt_node].prev[0] == "K" and neighb[0] == "J":
                            continue
                        queue.append(neighb)
                        graph["nodes"][neighb].prev = crt_node
                        graph["nodes"][crt_node].passed = True

                        if graph["nodes"][neighb].final == True:
                            dest = neighb

                if dest is not None:
                    break
            except KeyError:
                pass

        #### recreate path

        recreated_path = [dest]

        while graph["nodes"][recreated_path[-1]].prev != "-":
            recreated_path.append(graph["nodes"][recreated_path[-1]].prev)

        print(recreated_path)

        recreated_path = [rn[0] for rn in recreated_path]

        return recreated_path[::-1]

    def update_neighb(self, node: Node, section: int):
        new_node = copy.deepcopy(node)
        new_neighbors = [n[0] + "_" + str(section) for n in node.neighbors]
        new_node.neighbors = new_neighbors
        return new_node

    def develop_level(self, base_graph, tasks):
        if len(tasks) == 0:
            base_graph["nodes"]["0_" + str(self.num_sections)].final = True
            return copy.deepcopy(base_graph)
        upper_graphs = []
        for code in range(1, 2 ** (len(tasks))):
            binary_mask = [int(c) for c in bin(code)[2:]]
            binary_mask = [0] * (len(tasks) - len(binary_mask)) + binary_mask
            tasks_subset = np.array(tasks)[np.array(binary_mask) == 1]

            edges_interm = [(k, e) for k, e in base_graph["edges"].items() if
                            collections.Counter(e.tasks) == collections.Counter(tasks_subset)]

            if len(edges_interm) == 0:
                continue

            section = self.num_sections + 1
            self.num_sections += 1

            """for k, e in edges_interm:
                base_graph["edges"].pop(k)
                base_graph["nodes"][e.x].neighbors.remove(e.y)"""

            upper_graph = {"nodes": {k[0] + "_" + str(section): self.update_neighb(node, section)
                                     for k, node in base_graph["nodes"].items()},
                           "edges": {k[:2] + "_" + str(section): copy.deepcopy(e)
                                     for k, e in base_graph["edges"].items() if k[-7:] != "_interm"}}

            for e in upper_graph["edges"].values():
                e.tasks = [task for task in e.tasks if task not in tasks_subset]
                e.x = e.x[0] + "_" + str(section)
                e.y = e.y[0] + "_" + str(section)

            for k, edg in edges_interm:
                e = copy.deepcopy(edg)
                e.y = e.y[0] + "_" + str(section)
                e.tasks = []
                upper_graph["edges"][k[:2] + "_" + str(section) + "_interm"] = e
                base_graph["nodes"][e.x].neighbors.append(e.y)

            upper_graph = self.develop_level(upper_graph, np.array(tasks)[np.array(binary_mask) == 0])
            if upper_graph is not None:
                upper_graphs.append(upper_graph)

        if len(upper_graphs) == 0:
            return None

        for upper_graph in upper_graphs:
            if upper_graph is None:
                continue
            base_graph["nodes"].update(upper_graph["nodes"])
            base_graph["edges"].update(upper_graph["edges"])
        return base_graph

    def construct_nodes_list(self, tasks_list):

        graph = {"nodes": copy.deepcopy(self.nodes),
                 "edges": copy.deepcopy(self.edges)}

        for k, e in graph["edges"].items():
            e.x = k[0]
            e.y = k[1]
            e.tasks = [task for task in e.tasks if task in tasks_list]

        graph = self.develop_level(graph, tasks_list)

        self.nodes_list = self.shortest_path_unweighted("0", graph)

    def current(self):
        crt_node = self.nodes[self.nodes_list[self.node_idx]]
        if self.node_idx < len(self.nodes_list) - 1:
            next_node = self.nodes[self.nodes_list[self.node_idx + 1]]
            edge = self.edges.get(self.nodes_list[self.node_idx - 1] + self.nodes_list[self.node_idx])
            if self.node_idx != 0:
                prev_node = self.nodes[self.nodes_list[self.node_idx - 1]]
                next_edge = self.edges.get(self.nodes_list[self.node_idx] + self.nodes_list[self.node_idx + 1])
                cv2.arrowedLine(self.image, prev_node.coords,
                                crt_node.coords, (0, 128, 128), 2)
                direction = compute_direction(edge, next_edge)
            else:
                direction = "forward"
        else:
            direction = "stop"
            edge = None
        print(direction)
        cv2.imshow("next edge", self.image)
        cv2.waitKey(0)
        return direction, self.nodes_list[self.node_idx], edge

    def next(self):
        if self.node_idx < len(self.nodes_list) - 1:
            self.node_idx += 1
        return self.current()
