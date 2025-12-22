#!/usr/bin/env python3
"""
Generator node: dynamically launches the Verification, Monitor, and Target (Simple) nodes
based on a YAML configuration file.

Usage:
  ros2 run verification_system generator --ros-args -p config_path:=/path/to/config.yaml
"""

import os
import yaml
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.actions import LogInfo
from launch.utilities import normalize_to_list_of_substitutions

import rclpy
from rclpy.node import Node as RclpyNode


class Generator(RclpyNode):
    def __init__(self):
        super().__init__("generator")
        self.declare_parameter("config_path", "")
        self.config_path = self.get_parameter("config_path").value

        if not self.config_path:
            raise RuntimeError("Missing 'config_path' parameter for generator node.")
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Config file not found: {self.config_path}")

        self.get_logger().info(f"Loaded config from: {self.config_path}")
        self.run_launch()

    def run_launch(self):
        with open(self.config_path, "r") as f:
            cfg = yaml.safe_load(f)

        # ---- extract configuration fields ----
        node_name = cfg.get("node_name", "/simple_node")
        pkg = cfg.get("package", "ros_simple_node_pkg")
        exe = cfg.get("executable", "simple_node")
        ns = cfg.get("namespace", "")
        args = cfg.get("arguments", [])
        init_params = cfg.get("initial_params", {"use_sim_time": False})
        wait_sec = cfg.get("wait_for_service_sec", 10.0)

        # First param from param_list sets initial "input" param for the Simple Node
        first_input = "a"
        if "param_list" in cfg and cfg["param_list"]:
            first = cfg["param_list"][0]
            if isinstance(first, dict) and "char" in first:
                first_input = "a" if first["char"].upper() == "A" else "b"

        # Add the initial 'input' parameter
        init_params["input"] = first_input

        # ---- LaunchDescription ----
        ld = LaunchDescription()

        # Target node (Simple Node)
        target_node = Node(
            package=pkg,
            executable=exe,
            namespace=ns,
            name=node_name.strip("/"),
            parameters=[init_params],
            arguments=args,
            output="screen",
        )

        # Verification Node
        verification_node = Node(
            package="verification_node_pkg",
            executable="verification_node",
            name="verification_node",
            parameters=[{"config_path": self.config_path}],
            output="screen",
        )

        # Graph Node
        graph_node = Node(
            package="verification_node_pkg",
            executable="graph_node",
            name="graph_node",
            parameters=[],
            output="screen",
        )

        # Monitor Node
        monitor_node = Node(
            package="verification_node_pkg",
            executable="monitor",
            name="monitor",
            parameters=[{"config_path": self.config_path}],
            output="screen",
        )

        ld.add_action(
            LogInfo(msg=f"Launching target node [{pkg}/{exe}] as {node_name}")
        )
        ld.add_action(target_node)
        ld.add_action(verification_node)
        ld.add_action(graph_node)
        ld.add_action(monitor_node)

        # ---- Run the launch service ----
        ls = LaunchService()
        ls.include_launch_description(ld)
        self.get_logger().info("Starting LaunchService with 3 nodes ...")
        ls.run()


def main():
    rclpy.init()
    node = Generator()
    try:
        rclpy.spin(node)  # runs until interrupted (Ctrl+C)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
