import rclpy
import rospy
from rclpy.node import Node
from verification_inf_pkg.msg import State

#Sample A: Input a sequence of characters in the set of {a, b} and return {true, false, waiting}.
class SampleA(Node):

    def __init__(self):
        super().__init__('input_sequence')
        self.declare_parameter('input', 'a')
        input: str = self.get_parameter('input').get_parameter_value().string_value
        self.declare_parameter('prev_input', '')
        prev_input: str = self.get_parameter('prev_input').get_parameter_value().string_value
        self.declare_parameter('output', 'waiting')
        self.state_publisher = self.create_publisher(
            msg_type=State,
            topic='state_topic',
            qos_profile=1)
        self.state_input_subscriber = self.create_subscription(
            msg_type=State,
            topic='input_topic',
            callback=self.listener_callback,
            qos_profile=1)


    def print_result(self):
        """Method that returns the result of the input sequence."""
        if self.prev_input == self.input:
            self.output = 'false'
            self.prev_input = ''  # Reset prev_input to allow for new input
        else:
            if self.prev_input == '':
                self.output =  'waiting'
            else:
                self.output = 'true'
            self.prev_input = self.input  # Update prev_input for next call
        # Publish the state message
        self.state_publisher.publish(State(output=self.output, input=self.input, prev_input=self.prev_input))   

        self.get_logger().info(self.output)

def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    try:
        rclpy.init(args=args)

        sampleA = SampleA()

        rclpy.spin(sampleA)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()