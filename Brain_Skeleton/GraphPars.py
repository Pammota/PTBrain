
import math

from pygraphml import GraphMLParser
import numpy as np
import sys
sys.path.append("../")

class GraphPars():
    def __init__(self):
        parser = GraphMLParser()
        graph = parser.parse("Competition_track.graphml")
        self.adj_list = {}
        my_adj_list = {}
        self.V = {}
        for node in graph.nodes():
            ID, d0, d1 = self.parse_node(node)
            self.V[ID] = [d0, d1]


        for edge in graph.edges():
            d2 = self.parse_edge(edge)
            self.add_edge(edge.node1, edge.node2, self.adj_list)
            ID1, d0, d1 = self.parse_node(edge.node1)
            ID2, d0, d1 = self.parse_node(edge.node2)
            self.add_seen_edge(ID1, ID2, my_adj_list, d2)

        #print(self.adj_list)

    def parse_node(self, node):
        #print(str(node))
        list1 = str(node).split("\n")
        cnt = 1
        ID, d0, d1 = 0, 0, 0
        for item in list1:
            if cnt != 2 and cnt <= 4:
                x, y = item.split(": ")
            if cnt == 1:
                ID = y
            elif cnt == 3:
                d0 = y
            elif cnt == 4:
                d1 = y
            cnt = cnt + 1
        return ID, float(d0), float(d1)

    def add_edge(self, node1, node2, adj_list):
        temp = []
        if node1 not in adj_list:
            temp.append(node2)
            adj_list[node1] = temp
        elif node1 in adj_list:
            temp.extend(adj_list[node1])
            temp.append(node2)
            adj_list[node1] = temp

    def add_seen_edge(self, ID1, ID2, adj_list, d2):
        temp = []
        if ID1 not in adj_list:
            temp.append([ID2, d2, 'f'])
            adj_list[ID1] = temp
        elif ID1 in adj_list:
            temp.extend(adj_list[ID1])
            temp.append([ID2, d2, 'f'])
            adj_list[ID1] = temp

    def parse_edge(self, edge):
        item1, item2, useless = str(edge).split("\n")
        useless, ID = item1.split(": ")
        useless, d2 = item2.split(": ")
        return(d2)


    def get_closest_id(self, coords):
        a, b = coords['x'], coords['y']
        distances = {ID : math.sqrt((a - x)**2 + (b - y)**2) for ID, [x, y] in self.V.items()}

        return min(distances, key=distances.get)


    def get_starting_position(self, coords):
        position = self.get_closest_id(coords)

        ### we return the id (letter id, i.e: our self-defined graph) of the starting inersection point

        if position in ["86", "77"]:
            return "0", "A"
        if position in ["78", "87", "45"]:
            return "A", "B"
        if position in ["80", "92", "93", "94", "68"]:
            return "A", "D"
        if position in ["44", "89", "79"]:
            return "B", "A"
        if position in ["42", "98", "99", "100", "4"]:
            return "B", "E"
        if position in ["40", "90", "54"]:
            return "B", "C"
        if position in ["51", "104", "105", "106", "36"]:
            return "C", "F"
        if position in ["53", "91", "41"]:
            return "C", "B"
        if position in ["52", "107", "108", "109", "35"]:
            return "F", "C"
        if position in ["33", "113", "6"]:
            return "F", "E"
        if position in ["31", "114", "115", "116", "117", "118","27"]:
            return "F", "I"
        if position in["3", "101", "102", "103", "43"]:
            return "E", "B"
        if position in ["1", "111", "70"]:
            return "E", "D"
        if position in ["7", "8", "15", "16"] or (int(position) <= 143 and int(position) >= 134):
            return "E", "H"
        if position in ["5", "112", "34"]:
            return "E", "F"
        if position in ["81", "97", "96", "95", "67"]:
            return "D", "A"
        if position in ["71", "124", "125", "126", "127", "128", "59"]:
            return "D", "G"
        if position in ["69", "110", "2"]:
            return "D", "E"
        if position in ["72", "129", "130", "131", "132", "133", "58"]:
            return "G", "D"
        if position in ["62"] or (int(position) >= 148 and int(position) <= 171) or (int(position) >= 198 and int(position) <= 230):
            return "G", "J"
        if position in ["60", "144", "14"]:
            return "G", "H"
        if position in ["13", "145", "61"]:
            return "H", "G"
        if position in ["17", "146", "25"]:
            return "H", "I"
        if position in ["26", "32", "119", "120", "121", "122", "123"]:
            return "I", "F"
        if position in ["24", "147", "18"]:
            return "I", "H"
        if position in ["22"] or (int(position) >= 288 and int(position) <= 301):
            return "I", "J"
        if position in ["23"] or (int(position) >= 272 and int(position) <= 287):
            return "J", "I"
        if position in ["63"] or (int(position) >= 172 and int(position) <= 197) or (int(position) >= 231 and int(position) <= 266):
            return "J", "G"



    def validate_intersection(self, coords):
        position = self.get_closest_id(coords)

        if position not in ["86", "87", "87", "45", "79", "89", "90", "54", "41", "91", "97", "81",
                            "94", "68", "103", "43", "4", "100", "52", "109", "36",
                            "106", "6", "113", "112", "34", "2", "110", "70", "111", "72", "133", "128",
                            "59", "63", "197", "61", "145", "144", "14", "18", "147", "146",
                            "25", "27", "118", "23", "287", "32", "123", "300",
                            "301", "229", "230", "341", "342", "466", "467", "468"]:
            return False
        return True

if __name__ == "__main__":
    gp = GraphPars()

    print("THE CLOSEST NODE IS: " + gp.get_starting_position((14.23, 2.05)))
