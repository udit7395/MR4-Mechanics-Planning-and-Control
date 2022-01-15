import csv
import numpy as np
import math

number_of_samples = 250
number_of_neighbours = 5

starting_point = [-0.5, -0.5]
goal_point = [0.5, 0.5]

obstacles = []

PLOTTING = True

if PLOTTING:
    from matplotlib import pyplot as plt
    fig = plt.figure()
    axis = fig.add_subplot(1, 1, 1)
    axis.grid()
    axis.set_ylim(-0.5, 0.5)
    axis.set_xlim(-0.5, 0.5)
    axis.axhline(0, color='black')
    axis.axvline(0, color='black')
    axis.plot([-0.5, -0.5, -0.5, 0.5])
    axis.scatter(starting_point[0], starting_point[1], color='g')
    axis.scatter(goal_point[0], goal_point[1], color='b')

class Node():
    def __init__(self, node_id, x, y):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.heuristic_cost = getEuclideanDistance([x, y], goal_point)
        self.neighbours = []
        self.edges = []
        self.parent_node_id = -1
        self.past_cost = np.inf
        self.estimated_total_cost = -1

    def updateEstimatedTotalCost(self):
        self.estimated_total_cost = self.past_cost + self.heuristic_cost

    def getEstimatedTotalCost(self):
        return self.estimated_total_cost

    def updatePastCost(self, past_cost):
        self.past_cost = past_cost

    def getPastCost(self):
        return self.past_cost

    def getOptimisticCost(self):
        return self.heuristic_cost

    def addNeighbour(self, node_id):
        self.neighbours.append(node_id)

    def getNeighbours(self):
        return self.neighbours

    def addEdge(self, node_id, weight):
        self.addNeighbour(node_id)
        self.edges.append([node_id, weight])

    def getEdges(self):
        return self.edges

    def getPosition(self):
        return [self.x, self.y]

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def getNodeId(self):
        return self.node_id

    def updateParentNode(self, parent_node_id):
        self.parent_node_id = parent_node_id

    def getParentNode(self):
        return self.parent_node_id

def getEuclideanDistance(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    return math.sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))

def findNeighboursForNode(search_node, nodes):
    distances = []
    for index in range(len(nodes)):
        dist = getEuclideanDistance(search_node.getPosition(), nodes[index].getPosition())
        distances.append([dist, nodes[index].getNodeId()])
    distances.sort()

    new_neighbour_counter = 0
    for i in range(1, len(distances)):
        if new_neighbour_counter == number_of_neighbours:
            break
        node = nodes[distances[i][1]]
        dist = distances[i][0]
        if not isLineInCollisionWithAnyObstacle(search_node.getPosition(), node.getPosition()):
            if not search_node.getNodeId() in node.getNeighbours():
                new_neighbour_counter += 1
            search_node.addEdge(node.getNodeId(), dist)

def isPointInCollisionWithAnyObstacle(x, y):
    for obstacle in obstacles:
        cx, cy, radius = obstacle
        if (x - cx) ** 2 + (y - cy) ** 2 <= (radius + 0.008) ** 2:
            return True
    return False

def isLineInCollisionWithAnyObstacle(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    m = (y2 - y1) / (x2 - x1)
    c = y2 - (x2 * m)
    k = 0.02
    while k <= 1.0:
        x = x1 + k * (x2 - x1)
        y = y1 + k * (y2 - y1)
        if isPointInCollisionWithAnyObstacle(x, y):
            return True
        k += 0.04
    return False

# Parse the obstacles.csv file
with open('../results/obstacles.csv') as obstacles_csv:
    obstacles_csv_reader = csv.reader(obstacles_csv, delimiter=',')
    for row in obstacles_csv_reader:
        if row[0][0] != '#':
            cx = float(row[0])
            cy = float(row[1])
            radius = float(row[2]) / 2.0
            obstacles.append([cx, cy, radius])
            if PLOTTING:
                axis.add_artist(plt.Circle((cx, cy), radius, color = 'r', alpha = 0.5))
                axis.scatter(cx, cy, color='#000000')

nodes = {}
nodes[0] = Node(0, starting_point[0], starting_point[1])
nodes[0].updatePastCost(0.0)

# Phase 1, sampling: using uniform random distribution over the square [-0.5, 0.5] x [-0.5, 0.5]
# number of samples to be generated is defined above
samples_x = np.random.uniform(-0.5, 0.5, number_of_samples)
samples_y = np.random.uniform(-0.5, 0.5, number_of_samples)
node_number = 1
for i in range(number_of_samples):
    x = samples_x[i]
    y = samples_y[i]
    if not isPointInCollisionWithAnyObstacle(x, y):
        nodes[node_number] = Node(node_number, x, y)
        if PLOTTING:
            axis.scatter(x, y, color='#FF4500', alpha = 0.5)
        node_number += 1

nodes[node_number] = Node(node_number, goal_point[0], goal_point[1])

#Phase 2, creating edges
# I used euclidean distance between nodes as paramenter for neighbours
# The number of neighbours each node can have is defined above (number_of_neighbours)
for i in range(len(nodes)):
    current_node = nodes[i]
    findNeighboursForNode(current_node, nodes)
    if PLOTTING:
        for neighbour_node_id in current_node.getNeighbours():
            plt.plot([current_node.getX(), nodes[neighbour_node_id].getX()],
                     [current_node.getY(), nodes[neighbour_node_id].getY()],
                      color = '#AADDFF', alpha = 0.5)


#Phase 3, searching the graph : using A* star
open_nodes = [(nodes[0].getOptimisticCost(), nodes[0].getNodeId())]
closed_nodes = []

while len(open_nodes) != 0:
    _, current_node_id = open_nodes.pop(0)
    closed_nodes.append(current_node_id)
    if current_node_id == nodes[len(nodes) - 1].getNodeId():
        break
    for neighbour_node_id, edge_weight in nodes[current_node_id].getEdges():
        if neighbour_node_id in closed_nodes:
            continue

        neighbour_node = nodes[neighbour_node_id]
        current_node = nodes[current_node_id]
        tentative_past_cost = current_node.getPastCost() + edge_weight
        if tentative_past_cost < neighbour_node.getPastCost():
            neighbour_node.updatePastCost(tentative_past_cost)
            neighbour_node.updateParentNode(current_node_id)
            neighbour_node.updateEstimatedTotalCost()
            open_nodes.append((neighbour_node.getEstimatedTotalCost(), neighbour_node_id))
            open_nodes.sort()

# Generating the final path
shortest_path = [nodes[len(nodes) - 1].getNodeId() + 1]
index = -1

while index != 0:
    if index == -1:
        current_node = nodes[len(nodes) - 1]
        parent_node = nodes[current_node.getParentNode()]
        index = parent_node.getNodeId()
        shortest_path.append(parent_node.getNodeId() + 1)

        if PLOTTING:
            plt.plot([current_node.getX(), parent_node.getX()], [current_node.getY(), parent_node.getY()], color = '#000000', alpha = 0.9)
    else:
        current_node = nodes[index]
        parent_node = nodes[current_node.getParentNode()]

        index = parent_node.getNodeId()
        shortest_path.append(parent_node.getNodeId() + 1)

        if PLOTTING:
            plt.plot([current_node.getX(), parent_node.getX()], [current_node.getY(), parent_node.getY()], color = '#000000', alpha = 0.9)

shortest_path.reverse()

# Writing shortest path, nodes and edges to their respective csv files

with open('../results/path.csv', 'w') as paths_file:
    writer = csv.writer(paths_file)
    writer.writerow(shortest_path)
    print("Save shortest path to path.csv (in results folder)")

with open('../results/nodes.csv', 'w') as nodes_file:
    writer = csv.writer(nodes_file)
    for index in range(len(nodes)):
        current_node = nodes[index]
        # node_id + 1 because python is zero-indexed
        writer.writerow([current_node.getNodeId() + 1, current_node.getX(), current_node.getY(), current_node.getOptimisticCost()])
    print("Saved nodes information to node.csv (in results folder)")

with open('../results/edges.csv', 'w') as edges_file:
    writer = csv.writer(edges_file)
    already_added_edge = []
    for index in range(len(nodes)):
        current_node = nodes[index]
        for neighbour_node_id, edge_weight in current_node.getEdges():
            if neighbour_node_id not in already_added_edge:
                # node_id + 1 because python is zero-indexed
                writer.writerow([current_node.getNodeId() + 1, neighbour_node_id + 1, edge_weight])
        already_added_edge.append(current_node.getNodeId())
    print("Saved edges information to edges.csv (in results folder)")

if PLOTTING:
    plt.show()