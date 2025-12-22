#!/usr/bin/env python3
import os
import json
from pathlib import Path
from typing import List, Dict, Any, Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from verification_inf_pkg.action import BuildGraph

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None  # type: ignore


class Monitor(Node):
    """
    - Subscribes to a topic carrying 'a'/'b' (default: /state_output)
    - Maintains and persists a state list: [{'id': N, 'currentState': ...}]
    - Sends a BuildGraph goal (id, symbol, config_path) to the Verification node
      on action name: 'build_graph'
    """

    def __init__(self):
        super().__init__('monitor')

        # ---- parameters
        self.declare_parameter('config_path', '')
        self.declare_parameter('state_file', '/tmp/state_list.json')
        # self.declare_parameter('topic_name', '/state_output')  # <<< make topic explicit

        self._config_path: str = self.get_parameter('config_path').value
        self._state_file: str = self.get_parameter('state_file').value

        # Load/validate YAML (so we fail early if it's wrong)
        if self._config_path:
            self.config = self._load_yaml(self._config_path)
        else:
            self.config = {}

        self._topic = f'{self.config.get('package')}/state_output'

        # ---- state tracking
        self._states: List[dict] = []
        self._id_counter: int = 0
        self._last_symbol: Optional[str] = None  # to form aa/ab/ba/bb
        self._busy: bool = False                  # simple back-pressure
        self._retry_timer = None

        # ---- subscriber
        # IMPORTANT: use the explicit topic param; do NOT derive from package name.
        self.create_subscription(String, self._topic, self._on_msg, 10)
        self.get_logger().info(f"Monitor subscribing to: {self._topic}")

        # ---- action client
        self._client = ActionClient(self, BuildGraph, 'build_graph')

    # ---------- helpers ----------

    def _save_states(self):
        try:
            path = Path(self._state_file).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            tmp = path.with_suffix(path.suffix + '.tmp')
            with tmp.open('w', encoding='utf-8') as f:
                json.dump(self._states, f, indent=2)
            tmp.replace(path)
            self.get_logger().info(f"Saved {len(self._states)} states to {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save state file: {e}")

    def _make_current_state(self, symbol: str) -> str:
        if self._last_symbol is None:
            return symbol  # first observation -> 'a' or 'b'
        return f"{self._last_symbol}{symbol}"  # aa/ab/ba/bb

    def _next_id(self) -> int:
        self._id_counter += 1
        return self._id_counter

    # ---------- subscriber callback ----------

    def _on_msg(self, msg: String):
        symbol = (msg.data or '').lower()
        if symbol not in ('a', 'b'):
            symbol = 'a'

        obs_id = self._next_id()
        current_state = self._make_current_state(symbol)

        # record & persist
        self._states.append({'id': obs_id, 'currentState': current_state})
        self._save_states()

        # remember last symbol
        self._last_symbol = symbol

        # send action goal (Fix A: futures + callbacks; no asyncio loop needed)
        if self._busy:
            # keep it simple: drop while in-flight (or replace with a small queue if needed)
            self.get_logger().warn('BuildGraph goal in-flight; dropping this event')
            return

        if not self._client.wait_for_server(timeout_sec=0.0):
            # retry once shortly if server not up yet
            self.get_logger().info('BuildGraph server not ready; retrying in 100 ms')
            if self._retry_timer is not None:
                try:
                    self._retry_timer.cancel()
                except Exception:
                    pass
            self._retry_timer = self.create_timer(
                0.1, lambda oid=obs_id, sym=symbol: self._retry_send(oid, sym)
            )
            return

        self._send_goal(obs_id, symbol)

    def _retry_send(self, obs_id: int, symbol: str):
        # one-shot: cancel timer
        try:
            self._retry_timer.cancel()
        except Exception:
            pass
        self._retry_timer = None

        if self._busy:
            self.get_logger().warn('BuildGraph goal in-flight; skipping retry')
            return
        if not self._client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('BuildGraph server still not ready; dropping retry')
            return

        self._send_goal(obs_id, symbol)

    # ---------- action send & callbacks ----------

    def _send_goal(self, state_id: int, symbol: str):
        goal = BuildGraph.Goal()
        goal.obs_id = state_id
        goal.symbol = symbol
        goal.config_path = self._config_path

        self._busy = True
        self.get_logger().info(f"Sending BuildGraph goal: id={state_id}, symbol={symbol}")
        fut = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('BuildGraph goal rejected')
            self._busy = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f"BuildGraph result: success={result.success} message='{result.message}'"
            )
        finally:
            self._busy = False

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(f"BuildGraph feedback: id={fb.obs_id}, edge='{fb.edge}'")

    # ---------- YAML loader (kept; robust paths) ----------

    def _load_yaml(
        self,
        yaml_path: str,
        package_name: Optional[str] = None,
        fallback_subdir: str = "config",
    ) -> Dict[str, Any]:
        candidates = []
        p = Path(yaml_path)

        if p.is_absolute():
            candidates.append(p)

        rel = p if not p.is_absolute() else Path(p.name)

        if get_package_share_directory and package_name:
            try:
                share_dir = Path(get_package_share_directory(package_name))
                candidates.append(share_dir / rel)
                if rel.parent == Path("."):
                    candidates.append(share_dir / fallback_subdir / rel)
            except Exception:
                pass

        here = Path(__file__).resolve()
        pkg_root = here.parents[1] if (len(here.parents) >= 2) else here.parent
        candidates.append(pkg_root / rel)
        if rel.parent == Path("."):
            candidates.append(pkg_root / fallback_subdir / rel)

        candidates.append(Path.cwd() / rel)

        chosen: Optional[Path] = next((c for c in candidates if c.is_file()), None)
        if not chosen:
            pretty = "\n  - ".join(str(c) for c in candidates)
            raise FileNotFoundError(f"Config YAML not found. Looked in:\n  - {pretty}")

        with chosen.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        # Validate expected keys (loose—monitor only needs config_path to pass along)
        if "node_name" not in data:
            self.get_logger().warn("YAML missing 'node_name' (monitor only passes it along)")
        if "param_list" not in data:
            self.get_logger().warn("YAML missing 'param_list' (monitor only passes it along)")

        return data


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)  # single-threaded
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
