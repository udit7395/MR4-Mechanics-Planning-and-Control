import csv
import numpy as np

# Initializa Lists required for A*
input_size = 12
past_cost = [np.inf] * input_size
past_cost[0] = 0.0

optimistic_cost = []
estimated_tot_cost = []
parent_node = [-1] * input_size

adjacency_matrix = np.zeros((input_size, input_size), dtype ='float') # this matrix is used to represent connections between the nodes(edges)

# Extract required information from the csv's
# nodes.csv and edges.csv are read from the results folder to avoid duplication
with open('../results/nodes.csv') as nodes_csv:
    node_csv_reader = csv.reader(nodes_csv, delimiter=',')
    for index, row in enumerate(node_csv_reader):
        if row[0][0] != '#':
            node_id = int(row[0])
            heuristic_cost = float(row[3])

            optimistic_cost.append(heuristic_cost)

            if node_id == 1:
                estimated_tot_cost.append(heuristic_cost)
            else:
                estimated_tot_cost.append(np.inf)

with open('../results/edges.csv') as edges_csv:
    edges_csv_reader = csv.reader(edges_csv, delimiter=',')
    for index, row in enumerate(edges_csv_reader):
        if row[0][0] != '#':
            id1 = int(row[0])
            id2 = int(row[1])
            edge_weight = float(row[2])
            adjacency_matrix[id1 - 1][id2 - 1] = edge_weight
            adjacency_matrix[id2 - 1][id1 - 1] = edge_weight

open_nodes = [(optimistic_cost[0], 0)]
goal = input_size - 1 # since Python uses zero-based indexing
closed_nodes = []

while len(open_nodes) != 0:
    _, current_node = open_nodes.pop(0)
    closed_nodes.append(current_node)
    if current_node == goal:
        break
    for nbr, edge_weight in enumerate(adjacency_matrix[current_node]):
        if nbr in closed_nodes:
            continue
        if edge_weight == 0.0:
            continue
        tentative_past_cost = past_cost[current_node] + edge_weight
        if tentative_past_cost < past_cost[nbr]:
            past_cost[nbr] = tentative_past_cost
            parent_node[nbr] = current_node
            estimated_tot_cost[nbr] = past_cost[nbr] + optimistic_cost[nbr]
            open_nodes.append((estimated_tot_cost[nbr], nbr))
            open_nodes.sort()

# Pretty printing the final output
final_path = [goal + 1]
index = -1

while index != 0:
    if index == -1:
        index = parent_node[goal]
        final_path.append(index + 1)
    else:
        index = parent_node[index]
        final_path.append(index + 1)

final_path.reverse()
with open("../results/path.csv", 'w') as output:
    writer = csv.writer(output, delimiter=',')
    writer.writerow(final_path)
    print("Shortest Path ", final_path)
    print("Saved path.csv to results folder")
