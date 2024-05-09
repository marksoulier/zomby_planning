import rclpy
from rclpy.node import Node
import yaml
from geopy.distance import geodesic
from ament_index_python.packages import get_package_share_directory
import os
import numpy as np
import folium
from queue import Queue
from sensor_msgs.msg import NavSatFix
import time

def get_config_path() -> str:
    package_path = get_package_share_directory('zomby_planning')
    config_path = os.path.join(package_path, 'config', 'config_usu.yaml')
    return config_path

def plot_nodes_and_connections(nodes: list[dict]):
    '''
    plot_nodes_and_connections: Plots the nodes and connections on a map and saves it to an HTML file

    Args:
    - nodes: A list of dictionaries, each containing the

    Returns:
    - Nones
    '''
    # Assuming nodes is a list of dictionaries with lat, lon, name, and connections (which are indices of other nodes in the list)
    
    # Create a map object, centered on the average coordinates of the nodes
    avg_lat = sum(node['lat'] for node in nodes) / len(nodes)
    avg_lon = sum(node['lon'] for node in nodes) / len(nodes)
    map_obj = folium.Map(location=[avg_lat, avg_lon], zoom_start=12)

    # Add markers for each node
    for node in nodes:
        folium.Marker(
            location=[node['lat'], node['lon']],
            popup=node['name'],
            icon=folium.Icon(icon='cloud')
        ).add_to(map_obj)

        # Draw lines to connected nodes
        for conn_id in node['connections']:
            connected_node = nodes[conn_id-1]
            folium.PolyLine(
                locations=[[node['lat'], node['lon']], [connected_node['lat'], connected_node['lon']]],
                color="blue",
                weight=2.5,
                opacity=1
            ).add_to(map_obj)

    # Save the map to an HTML file
    map_obj.save('map.html')

def plot_nodes_and_connections_path(nodes, start_id, end_id, path):
    # Assuming nodes is a list of dictionaries with lat, lon, name, and connections (which are indices of other nodes in the list)
    
    # Create a map object, centered on the average coordinates of the nodes
    avg_lat = sum(node['lat'] for node in nodes) / len(nodes)
    avg_lon = sum(node['lon'] for node in nodes) / len(nodes)
    map_obj = folium.Map(location=[avg_lat, avg_lon], zoom_start=12)

    # Add markers for each node, with popup showing the ID
    for node in nodes:
        # Determine the color of the marker
        if node['id'] == start_id:
            marker_color = 'green'  # Start node
        elif node['id'] == end_id:
            marker_color = 'red'    # End node
        else:
            marker_color = 'blue'   # All other nodes

        popup_text = f"ID: {node['id']}<br>Name: {node['name']}"
        folium.Marker(
            location=[node['lat'], node['lon']],
            popup=folium.Popup(popup_text, parse_html=True),
            icon=folium.Icon(color=marker_color)
        ).add_to(map_obj)

    # Draw lines for the general connections in light blue
    for node in nodes:
        for conn_id in node['connections']:
            connected_node = next((n for n in nodes if n['id'] == conn_id), None)
            if connected_node:
                folium.PolyLine(
                    locations=[[node['lat'], node['lon']], [connected_node['lat'], connected_node['lon']]],
                    color="lightblue",
                    weight=2.5,
                    opacity=1
                ).add_to(map_obj)

    # Highlight the path with a different color
    for i in range(len(path) - 1):
        node = next((n for n in nodes if n['id'] == path[i]), None)
        next_node = next((n for n in nodes if n['id'] == path[i+1]), None)
        if node and next_node:
            folium.PolyLine(
                locations=[[node['lat'], node['lon']], [next_node['lat'], next_node['lon']]],
                color="darkred",
                weight=3,
                opacity=1
            ).add_to(map_obj)

    # Save the map to an HTML file
    map_obj.save('map_with_path.html')


def create_adjacency_graph(config):
    graph = {}
    id_to_name = {}
    id_to_lat_lon = {}  # New dictionary for mapping IDs to latitude and longitude
    for node in config['nodes']:
        node_id = node['id']
        node_name = node['name']
        lat = node['lat']
        lon = node['lon']
        id_to_name[node_id] = node_name
        id_to_lat_lon[node_id] = (lat, lon)  # Store the latitude and longitude
        graph[node_id] = {}
        for connection_id in node['connections']:
            distance = calculate_distance(node, next(item for item in config['nodes'] if item['id'] == connection_id))
            graph[node_id][connection_id] = distance
    return graph, id_to_name, id_to_lat_lon


def calculate_distance(node1, node2):
    # Node format: {'lat': 40.712776, 'lon': -74.005974}
    point1 = (node1['lat'], node1['lon'])
    point2 = (node2['lat'], node2['lon'])
    distance = geodesic(point1, point2).meters
    return distance

def read_config(file_path: str) -> dict:
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

### Breadth First Search
def bfs_path(adjacency_matrix, start_id, end_id):
    n = len(adjacency_matrix)  # Number of nodes in the matrix
    visited = [False] * n  # Track visited nodes
    parent = {start_id: None}  # Track the parent of each node for path reconstruction
    
    # Convert start and end id to index if your ids are not starting from 0 or not sequential
    # This step is necessary if your node ids do not match the indices directly
    
    queue = Queue()
    queue.put(start_id)
    visited[start_id] = True
    
    # BFS loop
    while not queue.empty():
        current_node = queue.get()
        
        if current_node == end_id:  # Path found
            break
        
        for neighbor in range(n):
            # Check if there is an edge and the neighbor hasn't been visited
            if adjacency_matrix[current_node][neighbor] != 0 and not visited[neighbor]:
                visited[neighbor] = True
                parent[neighbor] = current_node  # Record path
                queue.put(neighbor)
                
    # Reconstruct path from end_id to start_id
    path = []
    current_node = end_id
    while current_node is not None:
        path.append(current_node)
        current_node = parent.get(current_node)
    
    # Return the path in the correct order (from start to end)
    return path[::-1]

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info('Path Planner node has started.')
        self.publisher_ = self.create_publisher(NavSatFix, 'gps_coordinates_go', 10)
        config_path = get_config_path()
        config: dict = read_config(config_path)
        print("Config")
        print(config)
        adjacency_graph, self.id_to_name, self.id_to_lat_lon = create_adjacency_graph(config)
        plot_nodes_and_connections(nodes=config['nodes'])
        print("Adjacency Graph")
        print(adjacency_graph)

        #create adjancency graph
        # Assuming 'adjacency_graph' is your graph and 'n' is the number of nodes
        n = len(adjacency_graph)
        # Initialize a matrix of zeros
        adjacency_matrix = np.zeros((n, n))

        for node in adjacency_graph:
            for connected_node, distance in adjacency_graph[node].items():
                adjacency_matrix[node-1][connected_node-1] = distance
        print("Adjacency Matrix")
        print(adjacency_matrix)

        # Find shortest path from node 1 to node 5
        start_id = 1
        end_id = 35
        path = bfs_path(adjacency_matrix, start_id-1, end_id-1)
        print("Path")
        #increement path by 1 to match the node id
        path = [i+1 for i in path]
        print(path)
        #print the location names
        print("Path Names")
        for i in path:
            print(self.id_to_name[i])

        # Plot the nodes and connections with the path highlighted
        plot_nodes_and_connections_path(config['nodes'], start_id, end_id, path)

        #calulate total distance
        total_distance = 0
        for i in range(len(path)-1):
            total_distance += adjacency_matrix[path[i]][path[i+1]]
        print("Total Distance")
        print(total_distance)

        #publish the coordinates
        self.publish_coordinates(path)

    def publish_coordinates(self, path):
        # path is a list of node IDs
        for node_id in path:
            lat, lon =  self.id_to_lat_lon[node_id]
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
            msg.latitude = lat
            msg.longitude = lon
            # Additional NavSatFix fields can be filled here as needed
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published coordinates for node {node_id}: ({lat}, {lon})")
            #sleep for 2 seconds
            self.get_logger().info("Sleeping for 2 seconds")
            time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()