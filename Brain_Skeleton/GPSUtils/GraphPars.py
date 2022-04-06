from pygraphml import GraphMLParser
from pygraphml import Graph
import tempfile
import os
import sys
from queue import Queue
import matplotlib
import networkx as nx
sys.path.append("../../")

class GraphPars():
    def __init__(self):
        parser = GraphMLParser
        graph = parser.parse(self, "Competition_track.graphml")
        init_list = ['86', '45', '54', '36', '76']
        size = len(init_list) - 1
        adj_list = {}
        my_adj_list = {}
        V = {}
        E = []
        reversed_path = []
        for node in graph.nodes():
            ID, d0, d1 = self.parse_node(node)
            V[ID] = [d0, d1]
        for edge in graph.edges():
            E.append(edge)
            d2 = self.parse_edge(edge)
            self.add_edge(edge.node1, edge.node2, adj_list)
            ID1, d0, d1 = self.parse_node(edge.node1)
            ID2, d0, d1 = self.parse_node(edge.node2)
            self.add_seen_edge(ID1, ID2, my_adj_list, d2)
        while size > 0:
            self.bfs_between(init_list[size - 1], init_list[size], my_adj_list, reversed_path)
            size = size - 1
        reversed_path.append('86')
        self.set_directions(reversed_path, V)


    def parse_node(self, node):
        list1 = str(node).split("\n")
        cnt = 1
        for item in list1:
            if cnt != 2 and cnt <= 4:
                x,y = item.split(": ")
            if cnt == 1:
                ID = y
            elif cnt == 3:
                d0 = y
            elif cnt == 4:
                d1 = y
            cnt = cnt + 1
        return(ID, d0, d1)

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

    def bfs_between(self, node1, node2, adj_list, reversed_path):
        parrent = {}
        visited = {}
        i = 0
        while i <= 550:
            visited[str(i)] = '0'
            i = i + 1
        q = Queue(maxsize=5000)
        curr_node = node1
        parrent[node1] = node1
        q.put(curr_node)
        while curr_node != node2:
            curr_node = q.get()
            for node in adj_list[curr_node]:
                if visited[node[0]] == '0':
                    q.put(node[0])
                    parrent[node[0]] = curr_node
                    visited[node[0]] = '1'
        while parrent[curr_node] != curr_node:
            #print(curr_node)
            reversed_path.append(curr_node)
            curr_node = parrent[curr_node]
        #print(parrent[curr_node])

    def set_directions(self, reversed_path, V):
        size = len(reversed_path)
        i = size - 1
        while i >= 0:
            if i < size - 2:
                temp = V[reversed_path[i]]
                print(reversed_path[i], temp)
                if reversed_path[i] == '78':
                    if reversed_path[i+2] == '77':
                        a=0
                        # right
                elif reversed_path[i] == '80':
                    if reversed_path[i+2] == '79':
                        a=0
                        # right
                elif reversed_path[i] == '76':
                    if reversed_path[i+2] == '79':
                        a=0
                        # left
                elif reversed_path[i] == '44':
                    if reversed_path[i+2] == '43':
                        a=0
                        # right
                elif reversed_path[i] == '42':
                    if reversed_path[i+2] == '45':
                        a=0
                        # left
                elif reversed_path[i] == '42':
                    if reversed_path[i+2] == '41':
                        a=0
                        # right
                elif reversed_path[i] == '40':
                    if reversed_path[i+2] == '43':
                        a=0
                        # left
                elif reversed_path[i] == '51':
                    if reversed_path[i+2] == '54':
                        a=0
                        # left
                elif reversed_path[i] == '49':
                    if reversed_path[i+2] == '52':
                        a=0
                        # left
                elif reversed_path[i] == '31':
                    if reversed_path[i+2] == '34':
                        a=0
                        # left
                elif reversed_path[i] == '33':
                    if reversed_path[i+2] == '36':
                        a=0
                        # left
                elif reversed_path[i] == '33':
                    if reversed_path[i+2] == '32':
                        a=0
                        # right
                elif reversed_path[i] == '35':
                    if reversed_path[i+2] == '34':
                        a=0
                        # right
                elif reversed_path[i] == '1':
                    if reversed_path[i+2] == '4':
                        a=0
                        # left
                elif reversed_path[i] == '7':
                    if reversed_path[i+2] == '6':
                        a=0
                        # right
                elif reversed_path[i] == '5':
                    if reversed_path[i+2] == '4':
                        a=0
                        # right
                elif reversed_path[i] == '7':
                    if reversed_path[i+2] == '2':
                        a=0
                        # left
                elif reversed_path[i] == '3':
                    if reversed_path[i+2] == '6':
                        a=0
                        # left
                elif reversed_path[i] == '3':
                    if reversed_path[i+2] == '2':
                        a=0
                        # right
                elif reversed_path[i] == '71':
                    if reversed_path[i+2] == '70':
                        a=0
                        # right
                elif reversed_path[i] == '69':
                    if reversed_path[i+2] == '72':
                        a=0
                        # left
                elif reversed_path[i] == '69':
                    if reversed_path[i+2] == '68':
                        a=0
                        # right
                elif reversed_path[i] == '67':
                    if reversed_path[i+2] == '70':
                        a=0
                        # left
                elif reversed_path[i] == '62':
                    if reversed_path[i+2] == '61':
                        a=0
                        # right
                elif reversed_path[i] == '60':
                    if reversed_path[i+2] == '59':
                        a=0
                        # right
                elif reversed_path[i] == '60':
                    if reversed_path[i+2] == '63':
                        a=0
                        # left
                elif reversed_path[i] == '58':
                    if reversed_path[i+2] == '61':
                        a=0
                        # left
                elif reversed_path[i] == '13':
                    if reversed_path[i+2] == '16':
                        a=0
                        # left
                elif reversed_path[i] == '13':
                    if reversed_path[i+2] == '15':
                        a=0
                        # left
                elif reversed_path[i] == '17':
                    if reversed_path[i+2] == '16':
                        a=0
                        # right
                elif reversed_path[i] == '17':
                    if reversed_path[i+2] == '15':
                        a=0
                        # right
                elif reversed_path[i] == '24':
                    if reversed_path[i+2] == '27':
                        a=0
                        # left
                elif reversed_path[i] == '24':
                    if reversed_path[i+2] == '23':
                        a=0
                        # right
                elif reversed_path[i] == '22':
                    if reversed_path[i+2] == '25':
                        a=0
                        # left
                elif reversed_path[i] == '26':
                    if reversed_path[i+2] == '25':
                        a=0
                        # right
                else:
                    a=1
                    #forward
            i = i - 1