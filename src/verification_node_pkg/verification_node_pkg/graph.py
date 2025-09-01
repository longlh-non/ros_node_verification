import heapq
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

# add srv here
from verification_inf_pkg.srv.graph_srv import (
    AddNode,
    AddEdge,
    GetNeighbors,
    GetShortestPath,
)


class Graph(Node):
    def __init__(self):
        super().__init__("graph_node")
        self.get_logger().info("Graph node has been started.")

        # Graph parameters
        self.declare_parameter("directed", True)
        self._directed = self.get_parameter("directed").get_parameter_value().bool_value

        # Graph state
        self._next_id: int = 0
        self._adj: Dict[int, List[Tuple[int, int]]] = {}  # adjacency list
        self._pos: Dict[int, Tuple[float, float, float]] = {}  # node positions
        self._name_to_id: Dict[str, int] = {}  # mapping from names to ids

        # Services
        self._srv_add_node = self.create_service(
            AddNode, "graph/add_node", self.handle_add_node
        )
        self._srv_add_edge = self.create_service(
            AddEdge, "graph/add_edge", self.handle_add_edge
        )
        self._srv_neighbors = self.create_service(
            GetNeighbors, "graph/get_neighbors", self.handle_get_neighbors
        )
        self._srv_shortest = self.create_service(
            GetShortestPath, "graph/get_shortest_path", self.handle_get_shortest_path
        )

    # ---------------------- Service handlers ----------------------

    def handle_add_edge(self, request: AddEdge.Request, response: AddEdge.Response):
        """Method to add an edge to the graph."""
        u, v, w = request.u, request.v, float(request.w)
        if u not in self._adj or v not in self._adj:
            response.success = False
            response.message = f"One or both nodes {u}, {v} do not exist."
            self.get_logger().warn(response.message)
            return response

        self._adj[u].append((v, w))

        if request.undirected:
            self._adj[v].append((u, w))

        response.success = True
        response.message = f"Edge {u}->{v} (w={w})" + (
            ""
            if self._directed
            else (" (undirected mirrored)" if request.undirected else "")
        )
        self.get_logger().info(response.message)
        return response
    
    def handle_add_node(self, request: AddNode.Request, response: AddNode.Response):
        """Method to add a node to the graph."""
        name = request.name.strip()
        if name in self._name_to_id:
            response.id = self._name_to_id[name]
            self._pos[node_id] = (request.x, request.y, request.z)
            response.success = False
            response.message = f"Node '{name}' already exists with ID {response.id}."
            self.get_logger().warn(response.message)
            return response

        node_id = self._next_id
        self._next_id += 1
        self._name_to_id[name] = node_id
        self._pos[node_id] = (request.x, request.y, request.z)
        self._adj.setdefault(node_id, [])

        response.id = node_id
        response.success = True
        response.message = f"Node '{name}' added with ID {node_id} at position {self._pos[node_id]}."
        self.get_logger().info(response.message)
        return response
    
    def handle_get_neighbors(self, request: GetNeighbors.Request, response: GetNeighbors.Response):
        """Method to get neighbors of a node."""
        u = request.u
        nbrs = []
        if u in self._adj:
            nbrs = [v for v, _ in self._adj.get(u, [])]
        response.neighbors = nbrs
        self.get_logger().info(f"Neighbors of node {u}: {nbrs}")
        return response
    
    def handle_get_shortest_path(self, request: GetShortestPath.Request, response: GetShortestPath.Response):
        """Method to get the shortest path between two nodes using Dijkstra's algorithm."""
        src, dst = request.src, request.dst
        if src not in self._adj or dst not in self._adj:
            response.found = False
            response.path = []
            response.cost = float('inf')
            response.message = f"One or both nodes {src}, {dst} do not exist."
            self.get_logger().warn(response.message)
            return response
        
        dist, parent = self._dijkstra(src)
        if dist.get(dst, float('inf')) == float('inf'):
            response.found = False
            response.path = []
            response.cost = float('inf')
            response.message = f"No path from {src} to {dst}."
            self.get_logger().info(response.message)
            return response
        
        # Reconstruct path
        path = []
        cur = dst
        while cur != -1:
            path.append(cur)
            cur = parent.get(cur, -1)
        path.reverse()
        
        response.found = True
        response.path = path
        response.cost = float(dist[dst])
        return response
    
    # ---------------------- Algorithms ----------------------
    def _dijkstra(self, src: int):
        INF = float('inf')
        dist = Dict[int, float] = {u: INF for u in self._adj}
        parent: Dict[int, int] = {}
        dist[src] = 0.0
        pq: List[Tuple[float, int]] = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d != dist[u]:
                continue
            for v, w in self._adj[u]:
                nd = d + w
                if nd < dist.get(v, INF):
                    dist[v] = nd
                    parent[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, parent
    
    # TODO: Add more graph algorithms as needed


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        graph_node = Graph()
        rclpy.spin(graph_node)
    except KeyboardInterrupt:
        pass
    finally:
        graph_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
# from verification_node_pkg.srv import AddNode, AddEdge, GetNeighbors, GetShortestPath
