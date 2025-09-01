import rclpy
from rclpy.node import Node
from verification_inf_pkg.msg import State


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

        print_forever_node = SampleA()

        rclpy.spin(print_forever_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
