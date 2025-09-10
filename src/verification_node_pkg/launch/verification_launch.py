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
            ),
            Node(
                package="verification_node_pkg",
                executable="graph_node",
                name="graph_node",
                output="screen",
                parameters=[{"directed": True}],
            ),
        ]
    )
