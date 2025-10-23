#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


class SimpleNode(Node):
    """
    Publishes the value of parameter 'input' ('a' or 'b') to 'telemetry'.
    """
    def __init__(self):
        super().__init__('simple_node')

        # single required parameter for this node
        self.declare_parameter('input', 'a')  # 'a' or 'b'

        self._pub = self.create_publisher(String, 'telemetry', 10)

        # steady timer; message content is driven solely by the 'input' param
        self._timer = self.create_timer(0.5, self._on_timer)

        # allow runtime tweaking of 'input'
        self.add_on_set_parameters_callback(self._on_params_set)

    def _on_timer(self):
        val = str(self.get_parameter('input').value).lower()
        if val not in ('a', 'b'):
            val = 'a'
        msg = String(data=val)
        self._pub.publish(msg)

    def _on_params_set(self, params):
        # validate changes to 'input' param
        for p in params:
            if p.name == 'input':
                new_val = str(p.value).lower()
                if new_val not in ('a', 'b'):
                    # reject invalid values
                    return SetParametersResult(successful=False,
                                               reason="input must be 'a' or 'b'")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = SimpleNode()
    try:
        rclpy.spin(node)  # default SingleThreadedExecutor
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
