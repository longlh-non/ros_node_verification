from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="verification_node_pkg",
                executable="verification_node",
                name="verification_node",
                output="screen",
                parameters=[{"graph_ns": "graph_node", "config_path": "config.yaml"}],
            ),
            Node(
                package="verification_node_pkg",
                executable="graph_node",
                name="graph_node",
                output="screen",
                parameters=[{"directed": True}],
            ),
            Node(
                package="verification_node_pkg",
                executable="monitor",
                name="monitor",
                parameters=[
                    {
                        "target_node": "/my_target_node",
                        "watched_params": ["gain", "threshold", "mode"],
                        "publish_topic": "/state_sidecar/summary",
                        "auto_publish": True,
                    }
                ],
            ),
        ]
    )
