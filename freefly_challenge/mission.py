#!/usr/bin/env python3

__author__ = "Varundev Sukhil"
__contact__ = "vsukhil@gmail.com"

import os
import csv
import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from typing import List
from geometry_msgs.msg import Vector3
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from ament_index_python.packages import get_package_share_directory

class FreeflyMission(Node):
    """
    A new company has approached Freefly, and they want to use our Alta X drone to 3d print giant foam
    structures! They have a dispenser that mounts underneath the aircraft, and spits out foam that expands
    and hardens. What they need from Freefly is an application that can command the Alta X to fly to specific
    XYZ locations they specify in a text file so it can dispense foam along the way, building the structure.

    You, as the assigned drone engineer, need to create the following:
    A `Mavsdk` module written in C++ or python that can control a drone and fly an arbitrary pattern that is
    defined in a text file. The aircraft only needs to be able to fly in straight lines- any curves in the structure
    are turned into short straight segments by the customer's software.
    """

    def __init__(self) -> None:
        """declare all node workers and configs here"""
        
        # initialize the ROS2 node
        super().__init__("freefly_mission_node")

        # set QoS profile here (note: check using `ros2 topic echo <topic_name> --verbose`)
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 1)
        
        # read mission waypoints from file
        mission = "waypoints.csv"
        self.waypoints = self.read_mission_directive(mission = mission)
        
        # node topics for I/O
        self.control_mode = "/fmu/in/offboard_control_mode"
        self.goal_setpoint = "/fmu/in/trajectory_setpoint"
        self.vehicle_command = "/fmu/in/vehicle_command"
        self.local_position = "/fmu/out/vehicle_local_position"

        # node constants
        self.takeoff_height = -5.0
        self.offboard_stp_counter_thres = 10
        self.wpt_switch_thres = 0.5
        self.node_freq = 10
        self.wait_on_start = 20

        # node variables
        self.offboard_setpoint_counter = 0
        self.waypoint_iter = -1
        self.mission_start = False
        self.count_up = 0
        self.veh_loc_x = 0.0
        self.veh_loc_y = 0.0
        self.veh_loc_z = 0.0
        self.goal_loc_x = 0.0
        self.goal_loc_y = 0.0
        self.goal_loc_z = 0.0

        # node publishers
        self.veh_mode_pub = self.create_publisher(
            OffboardControlMode,
            self.control_mode,
            qos_profile = qos_profile)
        
        self.mission_goal_pub = self.create_publisher(
            TrajectorySetpoint,
            self.goal_setpoint,
            qos_profile = qos_profile)
        
        self.veh_cmd_pub = self.create_publisher(
            VehicleCommand,
            self.vehicle_command,
            qos_profile = qos_profile)
        
        # node subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            self.local_position,
            self.vehicle_local_position_callback,
            qos_profile = qos_profile)
        
        # main node spinner
        self.timer = self.create_timer(
            1.0 / self.node_freq,
            self.mission_node_callback)
    
    def read_mission_directive(self, mission: str) -> List[List[float]]:
        """load waypoints from the requested mission CSV file"""

        rel_path = os.path.join(get_package_share_directory("freefly_challenge"), "mission")
        waypoints = []
        with open(os.path.expanduser(f"{rel_path}/{mission}"), "r") as csv_file:
            csv_reader = csv.reader(csv_file)
            skip_first = False
            for row in csv_reader:
                if not skip_first:
                    skip_first = True
                else:
                    _, x, y, alt = map(float, row)
                    waypoints.append([x, y, alt])
        
        # add first waypoint to the end to complete the loop
        waypoints.append(waypoints[0])
        return(waypoints)

    def vehicle_local_position_callback(self, vehicle_local_position: VehicleLocalPosition) -> None:
        """read and store vehicle_local_position topic from subscriber"""

        self.veh_loc_x = vehicle_local_position.x
        self.veh_loc_y = -vehicle_local_position.y
        self.veh_loc_z = -vehicle_local_position.z

    def arm(self) -> None:
        """send an arm command to the vehicle"""

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1 = 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self) -> None:
        """send a disarm command to the vehicle"""

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1 = 0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self) -> None:
        """switch to offboard mode"""

        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1 = 1.0,
            param2 = 6.0)
        self.get_logger().info("switching to offboard mode")

    def land(self) -> None:
        """switch to land mode"""

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("switching to land mode")

    def publish_offboard_control_heartbeat_signal(self) -> None:
        """publish the offboard control mode"""

        cmd = OffboardControlMode()
        cmd.position = True
        cmd.velocity = True
        cmd.acceleration = False
        cmd.attitude = False
        cmd.body_rate = False
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.veh_mode_pub.publish(cmd)
    
    def publish_position_setpoint(self, x: float, y: float, alt: float) -> None:
        """publish the immediate next trajectory setpoint"""

        cmd = TrajectorySetpoint()
        cmd.velocity = [0.2, 0.2, 0.2]
        cmd.position = [x, y, alt]
        cmd.yaw = math.pi / 2.0  # (90 degree)
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mission_goal_pub.publish(cmd)
    
    def publish_vehicle_command(self, command, **params) -> None:
        """publish a vehicle command"""

        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = params.get("param1", 0.0)
        cmd.param2 = params.get("param2", 0.0)
        cmd.param3 = params.get("param3", 0.0)
        cmd.param4 = params.get("param4", 0.0)
        cmd.param5 = params.get("param5", 0.0)
        cmd.param6 = params.get("param6", 0.0)
        cmd.param7 = params.get("param7", 0.0)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000.0)
        self.veh_cmd_pub.publish(cmd)
    
    def log_term_waypoint(self) -> None:
        """for every iteration, print the goal pose and it's range"""

        self.get_logger().info("target: [{}, {}, {}] \t range: {}".format(
            self.goal_loc_x,
            self.goal_loc_y, 
            self.goal_loc_z,
            math.hypot(
                self.veh_loc_x - self.goal_loc_x,
                self.veh_loc_y + self.goal_loc_y,
                self.veh_loc_z + self.goal_loc_z)))

    def mission_node_callback(self) -> None:
        """callback function for the main node"""
        
        # establish heartbeat to the PX4 stack
        self.publish_offboard_control_heartbeat_signal()

        # arbitrary delay to allow visualizer artifacts to load
        if (not self.mission_start):
            
            # count up to the time delay constant
            if (self.count_up > self.node_freq * self.wait_on_start):
                self.mission_start = True
                self.engage_offboard_mode()
                self.arm()
                self.log_term_waypoint()

            self.count_up += 1

        else:

            # cycle through the individual waypoints in the mission
            if (self.waypoint_iter < len(self.waypoints)):

                # fetch the immediate next goal pose
                self.goal_loc_x = self.waypoints[(self.waypoint_iter + 1) % len(self.waypoints)][0]
                self.goal_loc_y = -self.waypoints[(self.waypoint_iter + 1) % len(self.waypoints)][1]
                self.goal_loc_z = -self.waypoints[(self.waypoint_iter + 1) % len(self.waypoints)][2]

                # check range to goal and switch waypoints at the range threshold
                if (math.hypot(
                    self.veh_loc_x - self.goal_loc_x,
                    self.veh_loc_y + self.goal_loc_y,
                    self.veh_loc_z + self.goal_loc_z) < self.wpt_switch_thres):
                    self.waypoint_iter += 1

                # publish info to the terminal and commands to the PX4 stack
                self.log_term_waypoint()
                self.publish_position_setpoint(self.goal_loc_x, self.goal_loc_y, self.goal_loc_z)

            # mission is complete!
            # land the drone and gracefully exit
            else:
                self.land()
                exit(0)

def main(args = None) -> None:
    """entry point for this node"""
    
    print("starting the mission node")

    # start the mission node
    rclpy.init(args = args)
    node = FreeflyMission()
    rclpy.spin(node)

    # destroy the mission node and gracefully exit
    node.destroy_node()
    rclpy.shutdown()