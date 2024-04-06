#!/usr/bin/env python3

__author__ = "Varundev Sukhil"
__contact__ = "vsukhil@gmail.com"

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

class FreeflyChallenge(object):
    """
    A simple launcher for the Freefly technical challenge.
    """

    def __init__(self) -> None:
        """declare all the nodes here"""

        rel_path = os.path.join(get_package_share_directory("freefly_challenge"), "config")

        self.rviz2_node = Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            arguments=['-d', [os.path.expanduser(f"{rel_path}/freefly.rviz")]])

        self.visualizer_node = Node(
            package = "freefly_challenge",
            executable = "visualizer",
            name = "visualizer")

        self.mission_node = Node(
            package = "freefly_challenge",
            executable = "mission",
            name = "mission")

def generate_launch_description() -> LaunchDescription:
    """assemble the nodes to launch and send to the server"""
    
    package = FreeflyChallenge()

    return LaunchDescription([
        package.rviz2_node,
        package.visualizer_node,
        package.mission_node])