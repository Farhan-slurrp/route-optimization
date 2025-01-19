import math
import os
import threading
import time
import requests
import heapq
import tkinter as tk
import tkintermapview
from dotenv import load_dotenv

load_dotenv()
mapbox_token = os.getenv('MAPBOX_ACCESS_TOKEN')
base_url = "https://api.mapbox.com/directions/v5/mapbox/driving"

start = (101.6958, 3.1466)
end = [(101.72311371383459, 3.155704236604605), (101.70858066181081, 3.146290771851916), (101.7099, 3.1441)]

def haversine(lat1, lon1, lat2, lon2):
    """Haversine formula to calculate the distance between two lat/lon points"""
    # Radius of the Earth in kilometers
    R = 6371.0

    # Convert degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Differences in coordinates
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # Distance in kilometers
    distance = R * c
    return distance

def clamp_coordinates(lat, lon):
    # Clamp latitude between -90 and 90, and longitude between -180 and 180
    lat = max(min(lat, 90), -90)
    lon = max(min(lon, 180), -180)
    return lat, lon
class RouteOptimization:
    def __init__(self, start: tuple, end: list[tuple]):
        # Keep track driver current position, Start at the start node
        self.current_position = start
        # Keep track driver current index in the path
        self.current_index = 0
        self.end_nodes = end
        self.graph = {}
        # Best path found so far
        self.current_best = None
        # Alternate (better) path found
        self.better_path = None
        # Flag to stop the driver simulation thread
        self.stop_thread = False
        # Placeholder for the map widget
        self.map_widget = None
        # Placeholder for the driver marker
        self.driver_marker = None
        # Placeholder for the path line on the map
        self.current_path_line = None
        # Placeholder for better path line on the map
        self.better_path_line = None

    def get_multiple_routes(self, start: tuple[int], end: tuple[int]):
        """Get routes between coordinates passed in the arguments"""
        url = f"{base_url}/{start[0]},{start[1]};{end[0]},{end[1]}?alternatives=true&steps=true&access_token={mapbox_token}&geometries=geojson"
        
        response = requests.get(url)
        
        if response.status_code == 200:
            data = response.json()
            return data['routes']
        else:
            print(f"Error fetching route data: {response.json()}")
            return None

    def get_closest_node(self, node, coordinates, max_distance=1.0):
        """Get the closest node from the graph to the given end node using the Haversine distance."""
        closest_node = None
        min_distance = float('inf')

        # Iterate through the coordinates to find the closest node
        for coor in coordinates:
            lat, lon = coor
            distance = haversine(node[0], node[1], lat, lon)
            if distance < min_distance:
                min_distance = distance
                closest_node = coor
        
        if min_distance > max_distance:
            return None

        return closest_node

    def dijkstra(self, start):
        """Dijkstra algorithm to get the shortest path from start to the destination"""
        # Priority queue for the shortest path calculation
        queue = [(0, start)]  # (time, node)
        times = {start: 0}
        previous_nodes = {start: None}

        # Process nodes in the priority queue
        while queue:
            current_time, current_node = heapq.heappop(queue)

            # Skip if the time is greater than the total stored time
            if current_time > times[current_node]:
                continue
            
            # Explore neighbors
            for neighbor, time in self.graph.get(current_node, []):
                time_taken = current_time + time
                if neighbor not in times or time_taken < times[neighbor]:
                    times[neighbor] = time_taken
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(queue, (time_taken, neighbor))

        # Find the last reachable node
        last_node = max(times, key=times.get)
        last_time = times[last_node]

        # Reconstruct the path to the last node
        path = []
        current_node = last_node
        while previous_nodes[current_node] is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        path.append(start)
        path = path[::-1]

        return path, last_time

    def create_graph_from_routes(self, routes):
        """Manipulate the graph with data from route"""
        for route in routes:
            prev_node = None
            prev_time = 0
            # Iterate through the steps in the route and build the graph
            for i in range(len(route['legs'][0]['steps'])):
                node = tuple(route['legs'][0]['steps'][i]['maneuver']["location"])
                time = prev_time + route['legs'][0]['steps'][i]['duration']
                if node not in self.graph:
                    self.graph[node] = []
                if prev_node is not None:
                    self.graph[prev_node].append((node, time))
                prev_node = node
                prev_time = time

    def find_shortest_route(self):
        """Find the shortest path from routes obtained from the API"""
        self.graph = {}
        for end_node in self.end_nodes:
            routes = self.get_multiple_routes(self.current_position, end_node)
            if routes:
                self.create_graph_from_routes(routes)
            else:
                print("Failed to fetch routes.")
            
        start_node = self.get_closest_node(self.current_position, self.graph)
        path, total_time = self.dijkstra(start_node)
        return path, total_time

    def update_driver_location(self):
        """Simulate driver movement and recompute path every 2 seconds"""
        while not self.stop_thread:
            path, total_time = self.find_shortest_route()
            print(f"Driver is at {self.current_position}")
            
            # If current_best is None, update it directly
            if self.current_best is None:
                self.current_best = (path, total_time)
                print(f"Initial current best path: {self.current_best[0]} with time: {self.current_best[1]}")
            else:

                # Check if new_best_path is part of the current_best
                if not self.is_subpath(path, self.current_best[0]):
                    # If the new route is heading to a new location
                    initial_end_node = self.get_closest_node(self.current_best[0][len(self.current_best[0])-1], 
                        self.end_nodes)
                    better_end_node = self.get_closest_node(path[len(path)-1], self.end_nodes)
                    if  initial_end_node != better_end_node:
                        print("We found you a new delivery point that closer than the delivery point you are heading to")
                    # If new_best_path is not part of current_best, update the better_path
                    print(f"New best path: {path} with time: {total_time}")
                    self.better_path = (path, total_time)
                    self.update_path_on_map(self.better_path[0], self.better_path_line, "blue")
                    print("Better path updated.")

                # Move to the next node (if there's any in current_best path)
                if len(self.current_best[0]) > 1 and self.current_index < len(self.current_best[0]):
                    # Move to the next node in the current best path
                    self.current_position = self.current_best[0][self.current_index]
                    self.current_index += 1
                    self.update_map_marker()  # Update the driver's position marker on the map
                    self.update_path_on_map(self.current_best[0], self.current_path_line, "red")  # Update the path on the map every time the position changes
                else:
                    print("Destination reached.")
                    self.stop_thread = True

            # Sleep for 2 seconds before re-evaluating
            time.sleep(2)

    def is_subpath(self, new_best_path, current_best_path):
        """Check if new_best_path is part of the current_best_path"""
        if not new_best_path or not current_best_path:
            return False
        # Check if new_best_path is a subpath of the current_best_path
        for i in range(len(current_best_path) - len(new_best_path) + 1):
            if current_best_path[i:i + len(new_best_path)] == new_best_path:
                return True
        return False

    def update_map_marker(self):
        """Update the driver's position marker on the map"""
        lon, lat = self.current_position
        self.driver_marker.set_position(lat, lon)

    def update_path_on_map(self, path, var, colour):
        """Update the path visualization on the map"""
        if var:
            var.delete() # Remove the previous path
        # Draw the current path as a polyline on the map
        path_coords = [(lat, lon) for lon, lat in path]
        var = self.map_widget.set_path(path_coords, color=colour, width=3)

    def setup_map(self):
        """Initialize tkinter window and map widget"""
        root = tk.Tk()
        root.title("Driver Movement Visualization")

        # Create and configure the map widget
        self.map_widget = tkintermapview.TkinterMapView(root, width=800, height=600)
        self.map_widget.set_position(self.current_position[1], self.current_position[0])  # Initial position
        self.map_widget.set_zoom(14)

        # Create and place a marker at the driver's starting position
        self.driver_marker = self.map_widget.set_marker(self.current_position[1], self.current_position[0], text="Driver")
        self.map_widget.pack()

        return root

    def start_delivery(self):
        """Start the delivery simulation and driver movement in a separate thread"""
        driver_thread = threading.Thread(target=self.update_driver_location)
        driver_thread.start()

if __name__ == '__main__':
    # Initialize the route optimization with the starting point and possible destinations
    route_optimization = RouteOptimization(start, end)
    # Set up the map window with the initial driver's position
    map_window = route_optimization.setup_map()
    # Start the delivery simulation
    route_optimization.start_delivery()

    # Start the Tkinter event loop
    map_window.mainloop()
