import rclpy
from rclpy.node import Node
from verification_inf_pkg.msg import State
from verification_inf_pkg.srv.graph_srv import AddNode, AddEdge, GetShortestPath


# VerificationNode.
class VerificationNode(Node):

    def __init__(self):
        super().__init__("verification_node")
        self.declare_parameter("input", "a")
        input: str = self.get_parameter("input").get_parameter_value().string_value
        self.declare_parameter("prev_input", "")
        prev_input: str = (
            self.get_parameter("prev_input").get_parameter_value().string_value
        )
        self.input_sequence: list = []

        self.verification_server = self.create_service(
            self,
        )

        self.cli_add_node = self.create_client(AddNode, "graph/add_node")
        self.cli_add_node = self.create_client(AddEdge, "graph/add_edge")
        self.cli_shortest_path = self.create_client(GetShortestPath, "graph/shortest_path")

        for cli in [self.cli_add_node, self.cli_add_edge, self.cli_shortest_path]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for graph service...")

        # demo once on start up

    def _call(self, cli, req):
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def test_graph(self):
        self.destroy_timer(self.timer)
        # add nodes
        r1 = AddNode.Request()
        r1.name = "A"
        r1.x = r1.y = r1.z = 0.0
        r2 = AddNode.Request()
        r2.name = "B"
        r2.x = 1
        r2.y = r2.z = 1.0
        a = self._call(self.cli_add_node, r1)
        b = self._call(self.cli_add_node, r2)
        # add edge from a to b
        e = AddEdge.Request()
        e.u = a.id
        e.v = b.id
        e.w = 1.0
        e.undirected = False
        self._call(self.cli_add_edge, e)
        # shortest path from a to b
        s = GetShortestPath.Request()
        s.src = a.id
        s.dst = b.id
        ans = self._call(self.cli_shortest_path, s)
        self.get_logger().info(f"Shortest path from {a.id} to {b.id}: {ans.path}")

    def verify_node(self):
        """Method that verifies the input sequence."""
        if self.prev_input == "":
            self.output = "waiting"
        if self.prev_input == self.input:
            self.output = "true"
            self.backtrack_node()
        else:
            self.output = "false"

        self.get_logger().info(self.output)

        self.get_logger().info(f"Printed {self.state} times.")
        self.state = self.state + 1

    def backtrack_node(self):
        """Method that allows backtracking in the input sequence."""
        #
        self.input_sequence.pop()
        self.get_logger().info(f"Backtracked to: {self.input_sequence}")

    def add_new_state(self, new_state):
        """Method to add a new state to the input sequence."""
        # new_state:
        # val
        # output
        self.input_sequence.append(new_state)
        self.get_logger().info(
            f"Added new state: {new_state} to sequence: {self.input_sequence}"
        )

    def reset_node(self):
        """Method to reset the node to its initial state."""
        self.input_sequence = []
        self.get_logger().info("Node has been reset to initial state.")


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)
        
        node = VerificationNode()
        rclpy.spin(node)

        # print_forever_node = SampleA()

        # rclpy.spin(print_forever_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.destroy_node()
        rclpy.shutdown()
        print(e)


if __name__ == "__main__":
    main()
