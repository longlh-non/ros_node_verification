#!/usr/bin/env python3
import os
import json
import asyncio
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String

from verification_inf_pkg.action import BuildGraph


class Monitor(Node):
    """
    - Subscribes to 'telemetry'
    - Builds and persists a state list
    - Sends a BuildGraph goal (id, symbol, config_path) to Verification Node
    """
    def __init__(self):
        super().__init__('monitor')

        # parameters
        self.declare_parameter('config_path', '')
        self.declare_parameter('state_file', '/tmp/state_list.json')

        self._config_path: str = self.get_parameter('config_path').value
        self._state_file: str = self.get_parameter('state_file').value

        self._states: List[dict] = []
        self._id_counter: int = 0
        self._last_symbol: Optional[str] = None  # to construct aa/ab/ba/bb

        # subscriber
        self.create_subscription(String, 'telemetry', self._on_msg, 10)

        # action client
        self._client = ActionClient(self, BuildGraph, 'build_graph')

    def _save_states(self):
        try:
            with open(self._state_file, 'w') as f:
                json.dump(self._states, f, indent=2)
            self.get_logger().info(f"Saved {len(self._states)} states to {self._state_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save state file: {e}")

    def _make_current_state(self, symbol: str) -> str:
        if self._last_symbol is None:
            return symbol  # first observation -> 'a' or 'b'
        pair = f"{self._last_symbol}{symbol}"
        # normalize pair to one of aa, ab, ba, bb (already is)
        return pair

    def _on_msg(self, msg: String):
        symbol = (msg.data or '').lower()
        if symbol not in ('a', 'b'):
            symbol = 'a'

        self._id_counter += 1
        current_state = self._make_current_state(symbol)

        # record state
        state_obj = {'id': self._id_counter, 'currentState': current_state}
        self._states.append(state_obj)
        self._save_states()

        # remember last symbol for the next edge
        self._last_symbol = symbol

        # send action goal to verification node
        asyncio.get_event_loop().create_task(self._send_goal(self._id_counter, symbol))

    async def _send_goal(self, state_id: int, symbol: str):
        await self._client.wait_for_server()

        goal = BuildGraph.Goal()
        goal.id = state_id
        goal.symbol = symbol
        goal.config_path = self._config_path

        self.get_logger().info(f"Sending BuildGraph goal: id={state_id}, symbol={symbol}")
        goal_handle = await self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)

        if not goal_handle.accepted:
            self.get_logger().error("BuildGraph goal was rejected")
            return

        result = await goal_handle.get_result_async()
        ok = result.result.success
        self.get_logger().info(f"BuildGraph result: success={ok}, message='{result.result.message}'")

    def _feedback_cb(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(f"BuildGraph feedback: id={fb.id}, edge='{fb.edge}'")


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
