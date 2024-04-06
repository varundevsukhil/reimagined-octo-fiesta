#!/usr/bin/env python3

__author__ = "Varundev Sukhil"
__contact__ = "vsukhil@gmail.com"

import os
import csv
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import List
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, Vector3, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory

class FreeflyVisualizer(Node):
    """
    A simple visualizer for the Freefly technical challenge.
    """

    def __init__(self) -> None:
        """declare all node workers and configs here"""

        # initialize the ROS2 node
        super().__init__("freefly_visualizer_node")

        # set QoS profile here (note: check using `ros2 topic echo <topic_name> --verbose`)
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1)

        # read mission waypoints from file
        mission = "waypoints.csv"
        self.markers = MarkerArray()
        self.waypoints = self.read_mission_directive(mission = mission)
        self.assemble_mission_markers()

        # node topics for I/O
        self.veh_attitude = "/fmu/out/vehicle_attitude"
        self.veh_local_pose = "/fmu/out/vehicle_local_position"
        self.mission_topic = "/visualizer/mission"
        self.veh_pose_topic = "/visualizer/vehicle_pose"
        self.veh_odom_topic = "/visualizer/vehicle_path"

        # node publishers
        self.mission_pub = self.create_publisher(
            MarkerArray,
            self.mission_topic,
            qos_profile = qos_profile)
        
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped,
            self.veh_pose_topic,
            qos_profile = qos_profile)
        
        self.vehicle_path_pub = self.create_publisher(
            Path,
            self.veh_odom_topic,
            qos_profile = qos_profile)
        
        # node subscribers
        self.create_subscription(
            VehicleAttitude,
            self.veh_attitude,
            self.vehicle_attitude_callback,
            qos_profile = qos_profile)
        
        self.create_subscription(
            VehicleLocalPosition,
            self.veh_local_pose,
            self.vehicle_local_position_callback,
            qos_profile = qos_profile)
        
        # node variables
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()

        # node constants
        self.trail_size = 1000
        self.frame_id = "map"
        self.node_freq = 10
        
        # main node spinner
        self.create_timer(
            1.0 / self.node_freq, 
            self.visualizer_node)

    def read_mission_directive(self, mission: str) -> List[List[float]]:
        """load waypoints from the mission CSV file"""

        rel_path = os.path.join(get_package_share_directory("freefly_challenge"), "mission")
        waypoints = []
        with open(os.path.expanduser(f"{rel_path}/{mission}"), "r") as csv_file:
            csv_reader = csv.reader(csv_file)
            skip_first = False
            for row in csv_reader:
                if (not skip_first):
                    skip_first = True
                else:
                    _, x, y, alt = map(float, row)
                    waypoints.append([x, y, alt])
        return(waypoints)
    
    def assemble_mission_markers(self) -> None:
        """perform a one-time assembly of the waypoint markers"""

        self.markers = MarkerArray()
        for wpt in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = self.waypoints.index(wpt)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale = Vector3(x = 1.0, y = 1.0, z = 1.0)
            marker.color = ColorRGBA(r = 1.0, g = 1.0, b = 0.0, a = 1.0)
            marker.pose.position = Point(x = wpt[0], y = wpt[1], z = wpt[2])
            marker.pose.orientation = Quaternion(x = 0.0, y = 0.0, z= 0.0, w = 1.0)
            self.markers.markers.append(marker)
    
    def vector_2_pose_msg(self, frame_id: str, position: np.array, attitude: np.array) -> PoseStamped:
        """convert position and attitude to ROS2 message"""

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.orientation = Quaternion(
            x = attitude[1],
            y = attitude[2],
            z = attitude[3],
            w = attitude[0])
        pose_msg.pose.position = Point(
            x = position[0],
            y = position[1],
            z = position[2])
        return(pose_msg)
    
    def vehicle_attitude_callback(self, data: VehicleAttitude) -> None:
        """read the drone's attitude"""

        self.vehicle_attitude[0] = data.q[0]
        self.vehicle_attitude[1] = data.q[1]
        self.vehicle_attitude[2] = -data.q[2]
        self.vehicle_attitude[3] = -data.q[3]

    def vehicle_local_position_callback(self, data: VehicleLocalPosition) -> None:
        """read the drone's local position"""

        self.vehicle_local_position[0] = data.x
        self.vehicle_local_position[1] = -data.y
        self.vehicle_local_position[2] = -data.z

    def append_vehicle_path(self, msg) -> None:
        """add the current location to the odometry trace"""

        self.vehicle_path_msg.poses.append(msg)
        if (len(self.vehicle_path_msg.poses) > self.trail_size):
            del self.vehicle_path_msg.poses[0]

    def visualizer_node(self) -> None:
        """the main visualizer node callback"""

        # stitch the drone's odometry trace
        vehicle_pose_msg = self.vector_2_pose_msg(
            self.frame_id,
            self.vehicle_local_position,
            self.vehicle_attitude)
        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.append_vehicle_path(vehicle_pose_msg)

        # publish visualization artifacts
        self.mission_pub.publish(self.markers)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

def main(args = None) -> None:
    """entry point for this node"""
    
    print("starting the visualizer node")

    # start the mission node
    rclpy.init(args = args)
    node = FreeflyVisualizer()
    rclpy.spin(node)

    # destroy the mission node and gracefully exit
    node.destroy_node()
    rclpy.shutdown()