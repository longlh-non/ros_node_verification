import rclpy
from rclpy.node import Node
from verification_inf_pkg.msg import State
from verification_inf_pkg.srv import AddNode, AddEdge, GetShortestPath, DFS


class Scenario:
    def __init__(self, nodes, edges, queries):
        # nodes:   List[Tuple[name, x, y, z]]
        # edges:   List[Tuple[u_name, v_name, weight, undirected_bool]]
        # queries: List[Tuple[src_name, dst_name]]
        self.nodes = nodes
        self.edges = edges
        self.queries = queries


# VerificationNode.
class VerificationNode(Node):

    def __init__(self):
        super().__init__("verification_node")
        # self.declare_parameter("input", "a")
        # input: str = self.get_parameter("input").get_parameter_value().string_value
        # self.declare_parameter("prev_input", "")
        # prev_input: str = (
        #     self.get_parameter("prev_input").get_parameter_value().string_value
        # )
        # self.input_sequence: list = []

        # talk to your server using your naming
        self.cli_add_node = self.create_client(AddNode, "/graph_node/add_node")
        self.cli_add_edge = self.create_client(AddEdge, "/graph_node/add_edge")
        # self.cli_sp = self.create_client(GetShortestPath, "/graph_node/shortest_path")
        self.cli_dfs = self.create_client(DFS, "/graph_node/dfs")

        for nm, cli in [
            ("add_node", self.cli_add_node),
            ("add_edge", self.cli_add_edge),
            # ("shortest_path", self.cli_sp),
            ("dfs", self.cli_dfs),
        ]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for /graph_node/{nm} ...")

        self.name2id = {}
        self._pending = 0
        self.scenario = None

    # ---------- public entry ----------
    def run(self, scenario: Scenario):
        self.scenario = scenario
        self._stage_add_nodes()

    # ---------- Stage 1: add N nodes (parallel) ----------
    def _stage_add_nodes(self):
        nodes = self.scenario.nodes
        if not nodes:
            self._stage_add_edges()
            return
        self.get_logger().info(f"Adding {len(nodes)} nodes...")
        self._pending = len(nodes)
        for name, x, y, z in nodes:
            req = AddNode.Request()
            req.name = name
            req.x = float(x)
            req.y = float(y)
            req.z = float(z)
            fut = self.cli_add_node.call_async(req)
            fut.add_done_callback(lambda f, name=name: self._on_node(name, f))

    # done-callback for add_node
    def _on_node(self, name, fut):
        try:
            res = fut.result()
            if res and res.ok:
                self.name2id[name] = res.id
                self.get_logger().info(f"Node '{name}' -> id {res.id}")
            else:
                self.get_logger().error(f"AddNode({name}) failed")
        except Exception as e:
            self.get_logger().exception(f"AddNode({name}) exception: {e}")
        finally:
            self._pending -= 1
            if self._pending == 0:
                self._stage_add_edges()

    # ---------- Stage 2: add M edges (parallel) ----------
    def _stage_add_edges(self):
        edges = self.scenario.edges
        if not edges:
            self._stage_dfs()
            return
        self.get_logger().info(f"Adding {len(edges)} edges...")
        self._pending = len(edges)
        for u_name, v_name, w, undirected in edges:
            u = self.name2id.get(u_name)
            v = self.name2id.get(v_name)
            if u is None or v is None:
                self.get_logger().error(f"Unknown node in edge {u_name}->{v_name}")
                self._pending -= 1
                continue
            req = AddEdge.Request()
            req.u = u
            req.v = v
            req.w = float(w)
            req.undirected = bool(undirected)
            fut = self.cli_add_edge.call_async(req)
            fut.add_done_callback(self._on_edge)

    def _on_edge(self, fut):
        try:
            res = fut.result()
            if not res or not res.ok:
                self.get_logger().error(
                    f"AddEdge failed: {getattr(res,'message','no message')}"
                )
        except Exception as e:
            self.get_logger().exception(f"AddEdge exception: {e}")
        finally:
            self._pending -= 1
            if self._pending == 0:
                self._stage_dfs()

    # def _stage_queries(self):
    #     queries = self.scenario.queries
    #     if not queries:
    #         self.get_logger().info("Scenario complete (no queries).")
    #         return
    #     self.get_logger().info(f"Running {len(queries)} queries...")
    #     for s_name, d_name in queries:
    #         s = self.name2id.get(s_name)
    #         d = self.name2id.get(d_name)
    #         if s is None or d is None:
    #             self.get_logger().error(f"Unknown node in query {s_name}->{d_name}")
    #             continue
    #         req = GetShortestPath.Request()
    #         req.src = s
    #         req.dst = d
    #         fut = self.cli_sp.call_async(req)
    #         fut.add_done_callback(
    #             lambda f, s_name=s_name, d_name=d_name: self._on_query(
    #                 s_name, d_name, f
    #             )
    #         )

    # def _on_query(self, s_name, d_name, fut):
    #     try:
    #         ans = fut.result()
    #         if ans and getattr(ans, "found", True):
    #             self.get_logger().info(
    #                 f"Path {s_name}->{d_name}: {list(getattr(ans,'path',[]))} (cost={getattr(ans,'cost',float('nan'))})"
    #             )
    #         else:
    #             self.get_logger().warn(f"No path {s_name}->{d_name}")
    #     except Exception as e:
    #         self.get_logger().exception(f"Query {s_name}->{d_name} exception: {e}")

    # def _call(self, cli, req, label):
    #     try:
    #         fut = cli.call_async(req)
    #         rclpy.spin_until_future_complete(self, fut)
    #         res = fut.result()
    #         if res is None:
    #             self.get_logger().error(
    #                 f"{label}: service returned None (server error?)"
    #             )
    #         else:
    #             self.get_logger().info(f"{label}: ok")
    #         return res
    #     except Exception as e:
    #         self.get_logger().exception(f"{label}: exception during call: {e}")
    #         return None

    # ---------- Stage 3: Run DFS (parallel) ----------
    def _stage_dfs(self):
        self.get_logger().info("Running DFS…")

        # Full traversal from 'A'
        a_id = self.name2id.get("A")
        if a_id is not None:
            req = DFS.Request()
            req.src = a_id
            req.dst = -1
            self.cli_dfs.call_async(req).add_done_callback(
                lambda f: self._on_dfs_full("A", f)
            )

        # Targeted DFS A -> D (optional)
        a_id = self.name2id.get("A")
        d_id = self.name2id.get("D")
        if a_id is not None and d_id is not None:
            req2 = DFS.Request()
            req2.src = a_id
            req2.dst = d_id
            self.cli_dfs.call_async(req2).add_done_callback(
                lambda f: self._on_dfs_to("A", "D", f)
            )

    def _on_dfs_full(self, src_name, fut):
        try:
            res = fut.result()
            self.get_logger().info(f"DFS from {src_name}: order={list(res.order)}")
        except Exception as e:
            self.get_logger().exception(f"DFS({src_name}) exception: {e}")

    def _on_dfs_to(self, s_name, d_name, fut):
        try:
            res = fut.result()
            if res.found:
                self.get_logger().info(f"DFS path {s_name}->{d_name}: {list(res.path)}")
            else:
                self.get_logger().warn(f"DFS could not reach {d_name} from {s_name}")
        except Exception as e:
            self.get_logger().exception(f"DFS({s_name}->{d_name}) exception: {e}")

    # def test_graph(self):
    #     self.get_logger().info("Testing graph node…")
    #     self.destroy_timer(self.timer)

    #     try:
    #         # add nodes
    #         r1 = AddNode.Request()
    #         r1.name = "A"
    #         r1.x = r1.y = r1.z = 0.0
    #         r2 = AddNode.Request()
    #         r2.name = "B"
    #         r2.x = 1.0
    #         r2.y = r2.z = 1.0

    #         a = self._call(self.cli_add_node, r1, "AddNode(A)")
    #         b = self._call(self.cli_add_node, r2, "AddNode(B)")
    #         if not a or not b:
    #             self.get_logger().error("AddNode failed; aborting.")
    #             return

    #         # add edge a -> b
    #         e = AddEdge.Request()
    #         e.u = a.id
    #         e.v = b.id
    #         e.w = 1.0
    #         e.undirected = False
    #         edge_res = self._call(self.cli_add_edge, e, "AddEdge(A->B)")
    #         if not edge_res or not edge_res.ok:
    #             self.get_logger().error(
    #                 f"AddEdge failed: {getattr(edge_res, 'message', 'no message')}"
    #             )
    #             return

    #         # shortest path a -> b
    #         s = GetShortestPath.Request()
    #         s.src = a.id
    #         s.dst = b.id
    #         ans = self._call(self.cli_shortest_path, s, "GetShortestPath(A,B)")
    #         if ans and getattr(ans, "found", True):  # if your srv has 'found'
    #             path = list(getattr(ans, "path", []))
    #             cost = getattr(ans, "cost", float("nan"))
    #             self.get_logger().info(
    #                 f"Shortest path from {a.id} to {b.id}: {path} (cost={cost})"
    #             )
    #         else:
    #             self.get_logger().warn("No path found A -> B")

    #     except Exception as e:
    #         self.get_logger().exception(f"test_graph crashed: {e}")

    # def verify_node(self):
    #     """Method that verifies the input sequence."""
    #     if self.prev_input == "":
    #         self.output = "waiting"
    #     if self.prev_input == self.input:
    #         self.output = "true"
    #         self.backtrack_node()
    #     else:
    #         self.output = "false"

    #     self.get_logger().info(self.output)

    #     self.get_logger().info(f"Printed {self.state} times.")
    #     self.state = self.state + 1

    # def backtrack_node(self):
    #     """Method that allows backtracking in the input sequence."""
    #     #
    #     self.input_sequence.pop()
    #     self.get_logger().info(f"Backtracked to: {self.input_sequence}")

    # def add_new_state(self, new_state):
    #     """Method to add a new state to the input sequence."""
    #     # new_state:
    #     # val
    #     # output
    #     self.input_sequence.append(new_state)
    #     self.get_logger().info(
    #         f"Added new state: {new_state} to sequence: {self.input_sequence}"
    #     )

    # def reset_node(self):
    #     """Method to reset the node to its initial state."""
    #     self.input_sequence = []
    #     self.get_logger().info("Node has been reset to initial state.")


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but used by ROS2 to configure
    certain aspects of the Node.
    """
    # try:
    #     rclpy.init(args=args)

    #     node = VerificationNode()
    #     rclpy.spin(node)

    #     # print_forever_node = SampleA()

    #     # rclpy.spin(print_forever_node)
    # except KeyboardInterrupt:
    #     pass
    # except Exception as e:
    #     node.destroy_node()
    #     rclpy.shutdown()
    #     print(e)
    rclpy.init()
    node = VerificationNode()

    # Example that scales: just change the data, not the logic
    scenario = Scenario(
        nodes=[
            ("A", 0, 0, 0),
            ("B", 1, 0, 0),
            ("C", 2, 0, 0),
            ("D", 3, 0, 0),
            ("E", 4, 0, 0),
        ],
        edges=[
            ("A", "B", 1.0, False),
            ("B", "C", 1.0, False),
            ("C", "D", 1.0, False),
            ("A", "E", 2.5, False),
            ("E", "D", 0.5, False),
        ],
        queries=[("A", "D"), ("B", "E")],
    )

    node.run(scenario)
    try:
        rclpy.spin(node)  # single-threaded, non-blocking
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
