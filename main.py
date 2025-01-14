import math
import os
import threading
import time
import requests
import heapq
from dotenv import load_dotenv

load_dotenv()
mapbox_token = os.getenv('MAPBOX_ACCESS_TOKEN')
base_url = "https://api.mapbox.com/directions/v5/mapbox/driving"

start = (101.6958, 3.1466)
end = [(101.7099, 3.1441), (101.72311371383459, 3.155704236604605), (101.70858066181081, 3.146290771851916)]

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


class RouteOptimization:
    def __init__(self, start: tuple, end: list[tuple]):
        # Keep track driver current position, Start at the start node
        self.current_position = start
        # Keep track driver current index in the path
        self.current_index = 0
        self.end_nodes = end
        self.graph = {}
        self.current_best = None
        self.better_path = None
        self.stop_thread = False

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

    def get_closest_node(self, node, coordinates, max_distance = 1.0):
        """Get the closest node from the graph to the given end node using the Haversine distance."""
        closest_node = None
        min_distance = float('inf')

        # Iterate through the graph and compute the distance to each node
        for coor in coordinates:
            lat, lon = coor
            distance = haversine(node[0], node[1], lat, lon)
            
            # Update the closest node if the distance is smaller
            if distance < min_distance:
                min_distance = distance
                closest_node = coor
        
        # If the closest node is further than the max_distance, return None
        if min_distance > max_distance:
            return None

        return closest_node

    def dijkstra(self, start):
        """Djikstra algorithm to get shortest path"""
        # Priority queue for the shortest path calculation
        queue = [(0, start)]  # (time, node)
        times = {start: 0}
        previous_nodes = {start: None}

        while queue:
            current_time, current_node = heapq.heappop(queue)

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
            prev = None
            for i in range(len(route['legs'][0]['steps'])):
                node = tuple(route['legs'][0]['steps'][i]['maneuver']["location"])
                time = route['legs'][0]['steps'][i]['duration']
                if node not in self.graph:
                    self.graph[node] = []
                if prev != None:
                    self.graph[prev].append((node, time))
                prev = node

    def find_shortest_route(self):
        """Find shortest path from routes obtained from API"""
        self.graph = {}
        for end_node in self.end_nodes:
            routes = self.get_multiple_routes(self.current_position, end_node)
            
            if routes:
                self.create_graph_from_routes(routes)
            else:
                print("Failed to fetch routes.")
            
        start_node = self.get_closest_node(self.current_position, self.graph)
                
        # Apply Dijkstra's algorithm to find the shortest path based on time
        path, total_time = self.dijkstra(start_node)
                
        if path:
            return path, total_time
        else:
            print("No path found.")

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
                if not self.is_subpath(path, self.current_best[0]) and total_time < self.current_best[1]:
                    # If the new route is heading to a new location
                    initial_end_node = self.get_closest_node(self.current_best[0][len(self.current_best[0])-1], 
                        self.end_nodes)
                    better_end_node = self.get_closest_node(path[len(path)-1], self.end_nodes)
                    if  initial_end_node != better_end_node:
                        print("We found you a new delivery point that closer than the delivery point you are heading to")
                    # If new_best_path is not part of current_best, update the better_path
                    print(f"New best path: {path} with time: {total_time}")
                    self.better_path = (path, total_time)
                    print("Better path updated.")

                # Move to the next node (if there's any in current_best path)
                if len(self.current_best[0]) > 1 and self.current_index < len(self.current_best[0]):
                    # Move to the next node in the current best path
                    self.current_position = self.current_best[0][self.current_index]
                    self.current_index += 1
                else:
                    print("Destination reached.")
                    self.stop_thread = True

            # Sleep for 2 seconds before re-evaluating
            time.sleep(2)

    def is_subpath(self, new_best_path, current_best_path):
        """Check if new_best_path is part of the current_best_path"""
        # Make sure both paths are not None and have length
        if not new_best_path or not current_best_path:
            return False

        for i in range(len(current_best_path) - len(new_best_path) + 1):
            if current_best_path[i:i + len(new_best_path)] == new_best_path:
                return True
        return False

    def start_delivery(self):
        """Start the delivery and driver movement in a separate thread"""
        # Start the pathfinding and driver movement in a separate thread
        driver_thread = threading.Thread(target=self.update_driver_location)
        driver_thread.start()

if __name__ == '__main__':
    route_optimization = RouteOptimization(start, end)
    route_optimization.start_delivery()
