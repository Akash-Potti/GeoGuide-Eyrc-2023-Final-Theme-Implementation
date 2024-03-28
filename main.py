######################### Path Planning And Geo Tracking #########################
'''
 * Team Id: 1792
 * Author List: Akash Potti
 * Filename: main.py
 * Theme: GeoGuide
 * Functions: create_graph(), find_shortest_paths(), plot_graph_with_shortest_paths(), send_command(), close_connection(), conver_list_string(), read_file(), priority_order(), main()
 * Global Variables: filtered_contours[], filtered_contours_x[], filtered_contours_y[], filtered_contours_w[], filtered_contours_h[]
 '''
######################## IMPORT MODULES #######################
import networkx as nx
import matplotlib.pyplot as plt
import socket
from itertools import groupby
import cv2
import numpy as np
import csv
import cv2.aruco as aruco
filtered_contours = []
filtered_contours_x = []
filtered_contours_y = []
filtered_contours_w = []
filtered_contours_h = []
# Function to create the graph

'''
* Function Name: create_graph
* Input: None
* Output: G (networkx.DiGraph) - Directed graph representing a network with nodes and weighted edges.
* Logic: This function creates a directed graph using the NetworkX library. It defines three sets of nodes representing three lanes: nodes_right, nodes_center, and nodes_left. Edges are defined between these nodes with associated weights indicating the distance or cost of traversal. The function then adds nodes and edges to the graph based on predefined connections and weights.
* Example Call: 
*  G = create_graph()
'''


def create_graph():
    # Initialize an empty directed graph
    G = nx.DiGraph()
    # Define nodes for three lanes: right, center, and left
    nodes_right = ['A', 'B', 'C', 'D']
    nodes_center = ['E', 'F', 'G', 'H']
    nodes_left = ['I', 'J', 'K']
    # Add nodes to the graph
    G.add_nodes_from(nodes_right + nodes_center + nodes_left)
    # Define edges between nodes with associated weights
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
                 'weight': 2}),
             ('G', 'I', {'weight': 2}), ('H', 'G', {
                 'weight': 1}), ('H', 'A', {'weight': 2}),
             ('H', 'I', {'weight': 3}), ('K', 'D', {
                 'weight': 4}), ('K', 'E', {'weight': 2}),
             ('K', 'J', {'weight': 1}), ('J', 'K', {
                 'weight': 1}), ('J', 'I', {'weight': 1}),
             ('J', 'F', {'weight': 2}), ('I', 'J', {
                 'weight': 1}),
             ('I', 'G', {'weight': 2}), ('A', '1', {'weight': 1}), ('G', '2', {'weight': 1}), ('2', 'I', {
                 'weight': 1}), ('I', '2', {'weight': 1}), ('2', 'G', {'weight': 1}),
             ('J', '3', {'weight': 1}), ('3', 'J', {'weight': 1}), ('F', '3', {
                 'weight': 1}), ('3', 'F', {'weight': 1}), ('F', '4', {'weight': 0.5}),
             ('4', 'F',  {'weight': 1}), ('C', '4', {'weight': 1}), ('4', 'C',  {
                 'weight': 1}), ('D', '5', {'weight': 1}), ('5', 'D', {'weight': 1}),
             ('K', '5', {'weight': 2}), ('5', 'K', {'weight': 2}), ('H', '1', {'weight': 1}), ('1', 'H', {'weight': 1}),]
    # Add edges to the graph
    G.add_edges_from(edges)
    # Return the created graph
    return G


'''
* Function Name: find_shortest_paths
* Input: graph (networkx.DiGraph) - Directed graph representing a network with nodes and weighted edges.
*        nodes (list) - List of nodes representing the sequence of locations to visit.
* Output: final_path (list) - List of nodes representing the shortest path to traverse all given nodes sequentially.
*         shortest_paths (dict) - Dictionary containing shortest paths from each source node to its corresponding target node.
* Logic: This function finds the shortest paths between consecutive nodes in the given sequence using Dijkstra's algorithm. It iterates through the list of nodes, computing the shortest path from each node to the next one in the sequence. The final path is constructed by concatenating these individual shortest paths. Additionally, the function stores all the computed shortest paths in a dictionary for reference.
* Example Call: 
*   graph = create_graph()
*   nodes_sequence = ['S', 'A', 'B', 'C', 'D', 'E']
*   shortest_path, shortest_paths_dict = find_shortest_paths(graph, nodes_sequence)
'''


def find_shortest_paths(graph, nodes):
    # Initialize variables to store the shortest paths and the final path
    shortest_paths = {}
    final_path = []
    # Iterate through the list of nodes
    for i in range(len(nodes) - 1):
        # Define source and target nodes for the current iteration
        source_node = nodes[i]
        target_node = nodes[i + 1]
        # Compute the shortest path using Dijkstra's algorithm
        path = nx.shortest_path(graph, source=source_node,
                                target=target_node, weight='weight')
        # Store the shortest path in the dictionary
        shortest_paths[target_node] = path
        # Extend the final path with the computed path, excluding the first node if not the first iteration
        final_path.extend(path[1:] if i > 0 else path)
        # Print the shortest path for visualization
        print(f"Shortest path from {source_node} to {target_node}: {path}")
    # Return the final path and the dictionary of shortest paths
    return final_path, shortest_paths


'''
* Function Name: plot_graph_with_shortest_paths
* Input: 
    - graph (networkx.Graph): The graph object representing the network.
    - pos_rotated (dict): A dictionary mapping nodes to their positions after rotation.
    - final_path (list): A list of nodes representing the shortest path to traverse all given nodes sequentially.
* Output: 
    - None
* Logic: 
    This function visualizes the graph with the shortest paths highlighted. It uses Matplotlib and NetworkX to plot the graph. The nodes are drawn with labels, node size, color, and font size specified. The edges representing the shortest path are drawn in red with a thicker width to distinguish them. Edge labels indicating the weights of the edges are also displayed.
* Example Call:
    plot_graph_with_shortest_paths(graph, pos_rotated, final_path)
'''


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


'''
* Function Name: send_command
* Input: 
    - client_socket (socket.socket): The socket object for communication with the client.
    - command (int): The command to be sent to the client.
* Output: 
    - None
* Logic: 
    This function sends a command to the client over the provided socket connection. The command is encoded as a string and then sent using the socket's sendall method. Additionally, it prints a message indicating the command being sent.
* Example Call:
    send_command(client_socket, command)
'''


def send_command(client_socket, command: int):
    client_socket.sendall(str.encode(str(command)))
    print("Sending Command", command)


'''
* Function Name: close_connection
* Input: 
    - client_socket (socket.socket): The socket object representing the client connection.
* Output: 
    - None
* Logic: 
    This function closes the provided socket connection with the ESP32 device. It calls the close method on the socket object to release the associated resources and terminates the connection. Additionally, it prints a message to indicate the closure of the connection.
* Example Call:
    close_connection(client_socket)
'''


def close_connection(client_socket):
    client_socket.close()
    print("Closed connection to ESP32")


'''
* Function Name: convert_list_string
* Input: 
    - final_path (list): The list of strings to be concatenated into a single string.
* Output: 
    - concatenated_string (str): The resulting string obtained by joining the elements of the input list.
* Logic: 
    This function takes a list of strings as input and concatenates all the strings into a single string. It utilizes the join method of strings to efficiently concatenate the elements. The resulting concatenated string is then returned.
* Example Call:
    concatenated_string = convert_list_string(final_path)
'''


def conver_list_string(final_path):
    return ''.join(final_path)


'''
* Function Name: read_file
* Input: 
    - file_path (str): The path to the file to be read.
* Output: 
    - identified_labels (dict): A dictionary containing key-value pairs read from the file.
* Logic: 
    This function reads data from a file located at the specified file_path. Each line of the file is assumed to contain key-value pairs separated by a colon (':'). The function iterates through each line, strips leading and trailing whitespaces, splits the line at the colon, and assigns the key-value pair to the identified_labels dictionary. Finally, the populated dictionary is returned.
* Example Call:
    identified_labels = read_file('identified_labels.txt')
'''


def read_file(file_path):
    identified_labels = {}
    with open(file_path, 'r') as file:
        for line in file:
            key, value = line.strip().split(':')
            identified_labels[key.strip()] = value.strip()

    return identified_labels


'''
* Function Name: priority_order
* Input: 
    - identified_labels (dict): A dictionary containing labels identified for each node.
* Output: 
    - prioritized_nodes (list): A list of nodes prioritized based on the labels.
* Logic: 
    This function prioritizes nodes based on the labels provided in the identified_labels dictionary. It first defines a priority order for the labels and a mapping of nodes to their corresponding labels. Then, it iterates through the nodes in the specified order ["A", "B", "C", "D", "E"]. For each node, it checks if there is a corresponding label in the identified_labels dictionary and if that label is in the priority list. If so, it calculates the priority index of the label and appends the node along with its priority index to the prioritized_nodes list. After processing all nodes, it sorts the prioritized_nodes list based on the priority index in descending order and extracts the node IDs, using the mapping dictionary, to form the final prioritized list of nodes. This list is then returned.
* Example Call:
    identified_labels = {'A': 'Fire', 'B': 'Combat', 'C': None, 'D': 'Destroyed buildings', 'E': 'Humanitarian Aid and rehabilitation'}
    prioritized_nodes = priority_order(identified_labels)
'''


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
* Function Name: main
* Input: None
* Output: None
* Logic: 
    - Creates a graph representing the environment using the create_graph function.
    - Defines the IP address for communication with ESP32.
    - Specifies the positions of nodes in the environment.
    - Reads identified labels from a file using the read_file function.
    - Determines the priority order of nodes based on identified labels using the priority_order function.
    - Constructs a sequence of nodes to visit, including the prioritized nodes and the starting node 'S'.
    - Finds the shortest paths between consecutive nodes in the sequence using the find_shortest_paths function.
    - Converts the obtained paths into a formatted list of nodes to send commands to the robot.
    - Establishes a socket connection with the ESP32 and sends the formatted path as commands.
    - Plots the graph with the shortest paths using the plot_graph_with_shortest_paths function.
    - Initializes ArUco marker detection and motion estimation.
    - Reads static ArUco marker data from a file.
    - Processes video frames captured from the camera, detects ArUco markers, estimates marker motion, and finds the closest marker to the moving marker.
    - Writes the closest marker ID to a CSV file.
    - Displays the video feed and terminates the loop upon pressing 'q'.
* Example Call: main()
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

    # plot_graph_with_shortest_paths(G, pos_rotated, final_list)

    # plot_graph_with_shortest_paths(G, pos_rotated, final_list)
    # Initialize ArUco marker detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()

    # Marker ID to track
    target_marker_id = 100
    x_frame = 400
    y_frame = 50
    width = 955
    height = 950

    # Initialize variables for motion estimation
    prev_marker_corners = None
    prev_rvecs, prev_tvecs = None, None
    cameraMatrix = np.eye(3)
    distCoeffs = np.zeros((4, 1))

    # Initialize video capture
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    # Static ArUco marker data
    static_data = []
    with open('lat_long.csv', 'r') as static_file:
        static_reader = csv.reader(static_file)
        next(static_reader)
        for row in static_reader:
            id, lat, long = map(float, row)
            static_data.append({'id': int(id), 'lat': lat, 'long': long})

    output_file_path = 'realtime_lat_long.csv'
    output_fieldnames = ['id', 'lat', 'long']

    try:
        while True:

            # Read frame from the camera
            ret, frame = cap.read()
            if not ret:
                break

            # Resize the frame
            frame = cv2.resize(frame, None, fx=2.25, fy=2.25)
            frame = frame[y_frame:y_frame + height, x_frame:x_frame + width]

            # Detect ArUco markers in the frame
            corners, ids, rejected = cv2.aruco.detectMarkers(
                frame, aruco_dict, parameters=parameters)

            # Initialize marker_corners
            marker_corners = None

            # Check if the target marker is detected
            if ids is not None and target_marker_id in ids:
                target_index = np.where(ids == target_marker_id)[0][0]
                marker_corners = corners[target_index][0]

                # Estimate marker motion
                if prev_marker_corners is not None:
                    try:
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            marker_corners, 0.05, cameraMatrix, distCoeffs)
                        if prev_rvecs is not None:
                            d_rvecs = rvecs - prev_rvecs
                            d_tvecs = tvecs - prev_tvecs
                            print("Relative rotation:", d_rvecs)
                            print("Relative translation:", d_tvecs)

                        prev_rvecs, prev_tvecs = rvecs, tvecs
                    except Exception as e:
                        pass

                # Draw marker outline and ID
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                # Find the closest marker to the moving marker
                if len(ids) > 1:  # At least one other marker detected
                    distances = []
                    for i, marker_id in enumerate(ids):
                        if marker_id != target_marker_id:  # Exclude the moving marker
                            distance = np.linalg.norm(marker_corners.mean(
                                axis=0) - corners[i][0].mean(axis=0))
                            distances.append((marker_id, distance))

                    closest_marker_id, min_distance = min(
                        distances, key=lambda x: x[1])
                    print(min_distance)
                    print("Closest marker to the moving marker:",
                          closest_marker_id)

                    # Write the closest marker ID to the CSV file
                    if min_distance < 90:
                        closest_marker_data = next(
                            (marker for marker in static_data if marker['id'] == closest_marker_id), None)
                        if closest_marker_data:
                            with open(output_file_path, 'w', newline='') as output_file:
                                writer = csv.DictWriter(
                                    output_file, fieldnames=output_fieldnames)
                                writer.writeheader()
                                writer.writerow(closest_marker_data)

            # Display the frame
            cv2.imshow('Frame', frame)

            # Break the loop when 'q' is pressedz
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Update previous marker corners
            prev_marker_corners = marker_corners

    finally:
        # Release video capture and close windows
        cap.release()
        cv2.destroyAllWindows()


# Entry point of the script
if __name__ == "__main__":
    main()
