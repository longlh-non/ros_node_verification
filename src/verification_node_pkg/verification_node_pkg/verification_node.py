#!/usr/bin/env python3
# Python
import os
import sys
import yaml
import asyncio
from typing import List, Dict, Any, Optional
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer
from rcl_interfaces.srv import SetParameters

# Custom pkgs
from verification_inf_pkg.srv import AddNode, AddEdge, GetShortestPath, DFS
from verification_inf_pkg.action import BuildGraph


def _ensure_abs_node_name(name: str) -> str:
    name = (name or "").strip()
    return name if name.startswith("/") else f"/{name}"


def _char_to_input(ch: str) -> str:
    """Map YAML char 'A'/'B' -> simple_node param 'input' = 'a'/'b'."""
    return "a" if str(ch).upper() == "A" else "b"


# VerificationNode.
class VerificationNode(Node):
    """
    Integrated node that:
      - Loads config (node_name, param_list, timing).
      - Provides BuildGraph action on 'verify':
          * feedback: {id, edge} where edge in {a,b,aa,ab,ba,bb}
          * applies next param_list entry (from index 1) to <node_name>.input
      - Keeps your graph service pipeline (AddNode/AddEdge/DFS).
    """

    def __init__(self, config_path: str):
        super().__init__("verification_node")

        # ---- Load YAML
        self.declare_parameter('config_path', '')
        self._config_path: str = self.get_parameter('config_path').value
        self.config = self._load_yaml(yaml_path=self._config_path)
        self.node_name: str = _ensure_abs_node_name(self.config["node_name"])
        self.param_list: List[Dict[str, Any]] = self.config.get("param_list", [])
        if not isinstance(self.param_list, list):
            raise RuntimeError("param_list must be a list of dictionaries")

        # Practical fields (present but not strictly required here)
        self.namespace: str = self.config.get("namespace", "")
        self.arguments: List[str] = self.config.get("arguments", [])
        self.initial_params: Dict[str, Any] = self.config.get("initial_params", {})

        self.apply_interval_sec: float = float(
            self.config.get("apply_interval_sec", 1.0)
        )
        self.wait_for_service_sec: float = float(
            self.config.get("wait_for_service_sec", 10.0)
        )

        # ---- Graph service clients (your original wiring)
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

        self.name2id: Dict[str, int] = {}
        self._pending: int = 0

        # ---- Verify action (key integration)
        # Expected goal fields: obs_id (uint64), input (string), output (string)
        self._verify_action = ActionServer(
            self, BuildGraph, "verify", execute_callback=self._verify_execute_cb
        )

        # Internal state for edge computation + parameter application index
        self._last_symbol: Optional[str] = None
        self._apply_index: int = 1  # start from *second* element in param_list

    # ---------- helpers ----------

    def _load_yaml(
        self,
        yaml_path: str,
        package_name: Optional[str] = "ros_simple_node_pkg",  # <-- set your package
        fallback_subdir: str = "config",  # e.g. <pkg>/config/params.yaml
    ) -> Dict[str, Any]:
        """
        Load a YAML file bundled with the project/package.

        Args:
            yaml_path: Either a filename (e.g., 'params.yaml' or 'config/params.yaml')
                       or an absolute path.
            package_name: ROS package to resolve share dir from (installed case).
            fallback_subdir: Subdirectory to look into under the package (source case).

        Returns:
            Parsed YAML as a dict.

        Raises:
            FileNotFoundError: If the YAML cannot be located in any known place.
            RuntimeError: If required keys are missing.
        """
        candidates = []
        print("yaml_path is here: ", yaml_path)
        p = Path(yaml_path)

        if p.is_absolute():
            candidates.append(p)

        # Normalize relative input like "config/params.yaml" vs "params.yaml"
        rel = p if not p.is_absolute() else p.name
        # 2) Installed package share directory: <install>/<pkg>/share/<pkg>/...
        if get_package_share_directory and package_name:
            try:
                share_dir = Path(get_package_share_directory(package_name))
                candidates.append(share_dir / rel)
                # also try <share>/<pkg>/<fallback_subdir>/<file> for convenience
                if not isinstance(rel, Path) or rel.parent == Path("."):
                    candidates.append(share_dir / fallback_subdir / rel)
            except Exception:
                pass  # not installed or package not found yet

        # 3) Source tree next to this file: <pkg>/<fallback_subdir>/<file>
        here = Path(__file__).resolve()
        pkg_root = here.parents[1] if (len(here.parents) >= 2) else here.parent
        candidates.append(pkg_root / rel)
        if not isinstance(rel, Path) or rel.parent == Path("."):
            candidates.append(pkg_root / fallback_subdir / rel)

        # 4) As a last resort, current working directory
        candidates.append(Path.cwd() / rel)

        # Pick the first existing file
        chosen: Optional[Path] = next((c for c in candidates if c.is_file()), None)
        if not chosen:
            pretty = "\n  - ".join(str(c) for c in candidates)
            raise FileNotFoundError(f"Config YAML not found. Looked in:\n  - {pretty}")

        with chosen.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if "node_name" not in data:
            raise RuntimeError("YAML must contain 'node_name'")
        if "param_list" not in data:
            raise RuntimeError("YAML must contain 'param_list'")

        return data

    async def _apply_next_param_set(self):
        """
        Applies param_list[self._apply_index] to <node_name>.input (as 'a' or 'b'),
        then increments index. No-op when list exhausted.
        """
        if self._apply_index >= len(self.param_list):
            self.get_logger().info("No more parameter sets to apply.")
            return

        entry = self.param_list[self._apply_index] or {}
        self._apply_index += 1

        desired = _char_to_input(entry.get("char", "A"))
        target_service = f"{self.node_name}/set_parameters"

        client = self.create_client(SetParameters, target_service)
        self.get_logger().info(f"Waiting for {target_service} ...")
        if not await client.wait_for_service(timeout_sec=self.wait_for_service_sec):
            self.get_logger().error(f"{target_service} not available.")
            return

        req = SetParameters.Request()
        req.parameters = [Parameter("input", value=desired).to_parameter_msg()]
        resp = await client.call_async(req)

        ok = all(r.successful for r in resp.results)
        if ok:
            self.get_logger().info(f"Applied {self.node_name}.input := '{desired}'")
        else:
            self.get_logger().error(f"Failed to apply {self.node_name}.input")

    # ---------- Verify action callback ----------

    async def _verify_execute_cb(self, goal_handle):
        """
        Build edge from last symbol and current 'input', send feedback,
        then apply next param_list entry (starting at index 1).
        """
        goal = goal_handle.request

        # Normalize symbol
        symbol = (goal.input or "").strip().lower()
        if symbol not in ("a", "b"):
            symbol = "a"

        # Compute edge
        edge = symbol if self._last_symbol is None else f"{self._last_symbol}{symbol}"

        # Feedback with id + edge
        fb = BuildGraph.Feedback()
        fb.id = int(goal.obs_id)
        fb.edge = edge
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f"[verify] id={fb.id} edge='{fb.edge}'")

        # Update last symbol
        self._last_symbol = symbol

        # Optional pacing
        if self.apply_interval_sec > 0:
            await asyncio.sleep(self.apply_interval_sec)

        # Apply next param set (skip first element as requested)
        await self._apply_next_param_set()

        # Result
        result = BuildGraph.Result()
        result.success = True
        result.message = f"Edge '{edge}' processed"
        goal_handle.succeed()
        return result

    # ---------- Stage 1: add N nodes (parallel) ----------
    def _stage_add_nodes(self):
        nodes = self.scenario.nodes
        if not nodes:
            self._stage_add_edges()
            return
        self.get_logger().info(f"Adding {len(nodes)} nodes...")
        self._pending = len(nodes)
        for name in nodes:
            req = AddNode.Request()
            req.name = name
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


def main(args=None):
    """
    Usage:
      ros2 run <your_pkg> verification_node <path/to/config.yaml>
    """
    # Expect path to YAML as first CLI arg or via environment SUPERVISOR_CONFIG
    cfg = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("SUPERVISOR_CONFIG")
    if not cfg:
        print("Usage: ros2 run <your_pkg> verification_node <path/to/config.yaml>")
        return

    # Ensure asyncio loop exists (important when used with rclpy)
    try:
        asyncio.get_event_loop()
    except RuntimeError:
        asyncio.set_event_loop(asyncio.new_event_loop())

    rclpy.init()
    node = VerificationNode(cfg)

    try:
        rclpy.spin(node)  # single-threaded, non-blocking
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
