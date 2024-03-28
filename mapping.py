import networkx as nx
import matplotlib.pyplot as plt
import socket
from itertools import groupby


def create_graph():
    G = nx.DiGraph()

    nodes_right = ['A', 'B', 'C', 'D']
    nodes_center = ['E', 'F', 'G', 'H']
    nodes_left = ['I', 'J', 'K']

    G.add_nodes_from(nodes_right + nodes_center + nodes_left)
    edges = [('S', 'A', {'weight': 0.5}), ('A', 'S', {'weight': 0.5}), ('A', 'B', {'weight': 1}), ('A', 'H', {'weight': 2}), ('B', 'C', {'weight': 1}),
             ('B', 'G', {'weight': 2}), ('B', 'A', {
                 'weight': 1}), ('C', 'B', {'weight': 1}),
             ('C', 'D', {'weight': 1}), ('C', 'F', {
                 'weight': 2}), ('D', 'C', {'weight': 1}),
             ('D', 'E', {'weight': 2}), ('D', 'K', {
                 'weight': 4}), ('E', 'D', {'weight': 2}),
             ('E', 'F', {'weight': 1}), ('E', 'K', {
                 'weight': 2}), ('F', 'E', {'weight': 1}),
             ('F', 'C', {'weight': 2}), ('F', 'G', {
                 'weight': 1}), ('F', 'J', {'weight': 2}),
             ('G', 'F', {'weight': 1}), ('G', 'B', {
                 'weight': 2}), ('G', 'H', {'weight': 1}),
             ('G', 'I', {'weight': 2}), ('H', 'G', {
                 'weight': 1}), ('H', 'A', {'weight': 2}),
             ('H', 'I', {'weight': 3}), ('K', 'D', {
                 'weight': 4}), ('K', 'E', {'weight': 2}),
             ('K', 'J', {'weight': 1}), ('J', 'K', {
                 'weight': 1}), ('J', 'I', {'weight': 1}),
             ('J', 'F', {'weight': 2}), ('I', 'J', {
                 'weight': 1}),
             ('I', 'G', {'weight': 2}), ('A', '1', {'weight': 1}), ('1', 'A', {
                 'weight': 1}), ('G', '2', {'weight': 1}), ('2', 'I', {'weight': 1}), ('I', '2', {'weight': 1}), ('2', 'G', {'weight': 1}),
             ('J', '3', {'weight': 1}), ('3', 'J', {'weight': 1}), ('F', '3', {
                 'weight': 1}), ('3', 'F', {'weight': 1}), ('F', '4', {'weight': 0.5}),
             ('4', 'F',  {'weight': 1}), ('C', '4', {'weight': 1}), ('4', 'C',  {
                 'weight': 0.5}), ('D', '5', {'weight': 2}),
             ('K', '5', {'weight': 2}), ('5', 'K', {'weight': 2}), ('H', '1', {'weight': 1})]

    G.add_edges_from(edges)

    return G


def find_shortest_paths(graph, nodes):
    shortest_paths = {}
    final_path = []

    for i in range(len(nodes) - 1):
        source_node = nodes[i]
        target_node = nodes[i + 1]
        path = nx.shortest_path(graph, source=source_node,
                                target=target_node, weight='weight')
        shortest_paths[target_node] = path
        final_path.extend(path[1:] if i > 0 else path)
        print(f"Shortest path from {source_node} to {target_node}: {path}")

    return final_path, shortest_paths


def plot_graph_with_shortest_paths(graph, pos_rotated, final_path):
    nx.draw(graph, pos_rotated, with_labels=True, font_weight='bold',
            node_size=700, node_color='lightblue', font_size=8)

    edges_shortest_path = [(final_path[i], final_path[i + 1])
                           for i in range(len(final_path) - 1)]
    nx.draw_networkx_edges(
        graph, pos_rotated, edgelist=edges_shortest_path, edge_color='red', width=2)

    edge_labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos_rotated, edge_labels=edge_labels)

    plt.show()


def send_command(client_socket, command: int):
    client_socket.sendall(str.encode(str(command)))
    print("Sending Command", command)


def close_connection(client_socket):
    client_socket.close()
    print("Closed connection to ESP32")


def conver_list_string(final_path):
    return ''.join(final_path)


def read_file(file_path):
    identified_labels = {}
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(':')
            identified_labels[key.strip()] = value.strip()

    return identified_labels


def priority_order(identified_labels):
    priority = ['Combat', 'Military Vehicles',
                'Humanitarian Aid and rehabilitation', 'Destroyed buildings', 'Fire']
    mapping = {"A": "1", "B": "2", "C": "3", "D": "4", "E": "5"}
    prioritized_nodes = []
    for key in ["A", "B", "C", "D", "E"]:
        if key in identified_labels:
            label = identified_labels[key]
            if label is not None and label in priority:
                priority_index = priority.index(label)
                prioritized_nodes.append((key, priority_index))

    print(prioritized_nodes)
    prioritized_nodes.sort(key=lambda x: x[-1], reverse=True)

    prioritized_nodes = [mapping[node[0]] for node in prioritized_nodes]
    return prioritized_nodes


'''
_____________
|           |
D-----E-----K
|     |     |
C-----F-----J 
|     |     |
B-----G---2--I
|     |      |
A--1---H-----|
'''


def main():
    G = create_graph()
    ip = "192.168.0.156"

    pos = {
        'S': (-1, 0), 'A': (0, 0), 'B': (1, 0), 'C': (2, 0), 'D': (3, 0),
        'E': (3, 1), 'F': (2, 1), 'G': (1, 1), 'H': (0, 1),
        'I': (1, 2), 'J': (2, 2), 'K': (4, 2), '1': (0, 0.5), '2': (1, 1.5), '3': (2, 1.5), '4': (2, 0.5), '5': (3.5, 1)
    }
    pos_rotated = {node: (y, x) for node, (x, y) in pos.items()}
    labels = read_file('identified_labels.txt')
    prioritized_nodes = priority_order(labels)

    nodes_to_visit = []
    nodes_to_visit.extend(prioritized_nodes)
    nodes_to_visit = ['S'] + nodes_to_visit + ['S']
    final_path_og, _ = find_shortest_paths(G, nodes_to_visit)
    final_list = []
    for i, c in enumerate(nodes_to_visit[:-1]):
        start = final_path_og.index(nodes_to_visit[i])
        end = final_path_og.index(nodes_to_visit[i+1])
        route = final_path_og[start:end+1]
        t_route = [x for x in route[1:-1] if x.isalpha()]
        p = list(route[0]) + t_route + list(route[-1])
        final_list.append(p)
        final_path_og = final_path_og[(end):]
    final_nodes = []
    for l in final_list:
        print(l)
        [final_nodes.append(x) for x in l]
    final_list = [i[0] for i in groupby(final_nodes)]

    print(f"Final path: {final_list}")
    final_path = conver_list_string(final_list)
    print(f"Final path: {final_list}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((ip, 8002))
        s.listen()
        client_socket, addr = s.accept()
        print("Connected to ESP32")

        with client_socket:
            send_command(client_socket, "_" + final_path)
            print("Sent path to ESP32")
    plot_graph_with_shortest_paths(G, pos_rotated, final_list)


if __name__ == "__main__":
    main()
