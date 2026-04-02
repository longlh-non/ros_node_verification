import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Edge:
    def __init__(self, input_val, to_state):
        self.input_val = input_val
        self.to_state = to_state


class Vertex:
    def __init__(self, state_id):
        self.state_id = state_id
        self.edges = []

    def add_edge(self, input_val, to_state):
        if not any(e.input_val == input_val for e in self.edges):
            self.edges.append(Edge(input_val, to_state))

    def get_next_state(self, input_val):
        for edge in self.edges:
            if edge.input_val == input_val:
                return edge.to_state
        return None


class FSMGraph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, state_id):
        if state_id not in self.vertices:
            self.vertices[state_id] = Vertex(state_id)

    def add_edge(self, from_state, input_val, to_state):
        self.add_vertex(from_state)
        self.add_vertex(to_state)
        self.vertices[from_state].add_edge(input_val, to_state)

    def get_next_state(self, current_state, input_val):
        if current_state not in self.vertices:
            return None
        return self.vertices[current_state].get_next_state(input_val)

    def print_graph(self, logger):
        logger.info("Current FSM graph:")
        for state_id, vertex in self.vertices.items():
            for edge in vertex.edges:
                logger.info(f"  State {state_id} --[{edge.input_val}]--> State {edge.to_state}")


class SimpleFSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # FSM internals
        self.graph = FSMGraph()
        self.current_state = 0
        self.input_stream = ['a', 'b', 'b', 'a', 'a', 'b']
        self.index = 0

        # Publisher: publishes current state
        self.state_publisher = self.create_publisher(String, 'fsm_state', 10)

        self.state_publisher = self.create_publisher(String, 'output_state', 10)

        self.state_change_subscriber = self.create_subscription(String, 'state_change', self.state_change_callback, 10)
        
        self.input_stream = self.create_subscription(String, 'input_stream', self.input_stream_callback, 10)

        # Timer: process one element per tick
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"FSM Node started. Initial state: {self.current_state}")
        self.get_logger().info(f"Input stream: {self.input_stream}")

    def derive_next_state(self, current_state, element):
        """Automatically derive the next state from current state and input."""
        if element == 'b':
            return 2
        elif element == 'a':
            return 0 if current_state == 1 else 1

    def state_change_callback(self, msg):
        self.get_logger().info(f"State change requested: {msg.data}")

    def input_stream_callback(self, msg):
        self.get_logger().info(f"Input: {msg.data}")


    # def timer_callback(self):
        # Stop when input stream is exhausted
        if self.index >= len(self.input_stream):
            self.get_logger().info("Input stream exhausted. FSM complete.")
            self.graph.print_graph(self.get_logger())
            self.timer.cancel()
            return

        element = self.input_stream[self.index]
        next_state = self.derive_next_state(self.current_state, element)

        # Add edge to graph if not already known
        self.graph.add_edge(self.current_state, element, next_state)

        self.get_logger().info(
            f"[{self.index}] State {self.current_state} --[{element}]--> State {next_state}"
        )
        
    
        # Transition
        self.current_state = next_state
        self.index = (self.index + 1) % len(self.input_stream)

        # Publish current state
        msg = String()
        msg.data = f"state:{self.current_state}"
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()