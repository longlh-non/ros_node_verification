import rclpy
from rclpy.node import Node as RosNode
from std_msgs.msg import String


class Edge:
    def __init__(self, label, to_node, output=None):
        self.label = label      # input symbol ('a'/'b') or state-change label ('sc=v')
        self.to_node = to_node
        self.output = output


class KripkeNode:
    def __init__(self, node_id, ap=None):
        self.node_id = node_id
        self.atomic_props = set(ap or [])
        self.edges = {}         # label -> Edge

    def add_edge(self, label, to_node, output=None):
        if label not in self.edges:
            self.edges[label] = Edge(label, to_node, output)
            if output:
                self.atomic_props.add(output)

    def get_edge(self, label):
        return self.edges.get(label)


class KripkeStructure:
    def __init__(self):
        self.nodes = {}
        self.initial_node = None
        self.logger = None

    def add_node(self, node_id, ap=None):
        if node_id not in self.nodes:
            self.nodes[node_id] = KripkeNode(node_id, ap)

    def add_edge(self, from_node, label, to_node, output=None):
        self.add_node(from_node)
        self.add_node(to_node)
        self.nodes[from_node].add_edge(label, to_node, output)

    def get_edge(self, current_node, label):
        if current_node not in self.nodes:
            return None
        return self.nodes[current_node].get_edge(label)

    @staticmethod
    def _derive_next_node(current_node, input_val):
        if current_node == 0:
            return 1 if input_val == 'a' else 2
        elif current_node == 1:
            return 0 if input_val == 'a' else 2
        elif current_node == 2:
            return 1 if input_val == 'a' else 0

    def _build_kripke_dfs(self):
        self.initial_node = 0
        self.add_node(0, ap={'in_state_0'})
        self._visited = {0}
        self.logger.info("[Kripke build] Added initial node 0")

    def _dfs(self, current, symbol, derive_output):
        next_node = self._derive_next_node(current, symbol)
        output = derive_output(current, symbol)

        is_new = next_node not in self._visited
        if is_new:
            self._visited.add(next_node)
            self.add_node(next_node, ap={f'in_state_{next_node}'})
            self.logger.info(f"[Kripke build] Discovered new state {next_node}")
            for existing_id in list(self.nodes.keys()):
                self.add_edge(existing_id, f'sc={next_node}', next_node)
                self.add_edge(next_node, f'sc={existing_id}', existing_id)
                self.logger.info(f"[Kripke build]   sc edge: State {existing_id} --[sc={next_node}]--> State {next_node}")

        if self.get_edge(current, symbol) is None:
            out_str = output if output else '∅'
            self.logger.info(f"[Kripke build] Edge: State {current} --[{symbol}]--> State {next_node}  output={out_str}"
                             + (" (back-edge)" if not is_new else ""))
            self.add_edge(current, symbol, next_node, output)

        return next_node

    def print_structure(self, logger):
        logger.info("Kripke Structure:")
        for node_id, node in self.nodes.items():
            state_props = {p for p in node.atomic_props if p.startswith('in_state_')}
            props = ', '.join(sorted(state_props)) if state_props else '∅'
            logger.info(f"  State {node_id}  AP={{{props}}}")
            for label, edge in node.edges.items():
                out_str = f" / {edge.output}" if edge.output else ""
                logger.info(f"    --[{label}{out_str}]--> State {edge.to_node}")


class SimpleKripkeNode(RosNode):
    def __init__(self):
        super().__init__('fsm_node')

        self.kripke = KripkeStructure()
        self.kripke.logger = self.get_logger()
        self.kripke._build_kripke_dfs()

        self.current_node = 0

        self.state_publisher = self.create_publisher(String, 'state', 10)
        self.output_publisher = self.create_publisher(String, 'output', 10)

        self.create_subscription(String, 'input', self.input_callback, 10)
        self.create_subscription(String, 'state_change', self.state_change_callback, 10)

        self._publish_state(self.current_node)
        self.get_logger().info(f"Simple Kripke Node started. Initial state: {self.current_node}")

    def _derive_output(self, current_node, input_val):
        if current_node == 1 and input_val == 'a':
            return 'YES'
        if current_node == 2 and input_val == 'b':
            return 'YES'
        return None

    def _publish_state(self, node):
        msg = String()
        msg.data = str(node)
        self.state_publisher.publish(msg)

    def input_callback(self, msg):
        self.get_logger().info(
            f"{msg}"
        )
        input_val = msg.data.strip()

        # Build Kripke graph from the symbol received from verification_node
        next_node = self.kripke._dfs(self.current_node, input_val, self._derive_output)

        output = self._derive_output(self.current_node, input_val)
        out_str = output if output else '∅'

        self.get_logger().info(
            f"State {self.current_node} --[{input_val}]--> State {next_node}  output={out_str}"
        )
        self.current_node = next_node
        self._publish_state(self.current_node)

        out_msg = String()
        out_msg.data = out_str
        self.output_publisher.publish(out_msg)

    def state_change_callback(self, msg):
        try:
            v = int(msg.data.strip())
        except ValueError:
            self.get_logger().warn(f"Invalid state_change value: '{msg.data}'")
            return

        edge = self.kripke.get_edge(self.current_node, f'sc={v}')
        if edge is None:
            self.get_logger().warn(f"Unknown state_change target: {v}")
            return

        self.get_logger().info(f"state_change: {self.current_node} -> {edge.to_node}")
        self.current_node = edge.to_node
        self._publish_state(self.current_node)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleKripkeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
