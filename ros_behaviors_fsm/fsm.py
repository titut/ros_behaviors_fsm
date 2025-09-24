"""Follow a Neato based on LIDAR Scan"""

from math import radians, cos, sin, sqrt, atan2

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

import numpy as np
from sklearn.cluster import DBSCAN


class NeatoFollower(Node):
    """Neato following node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the node. No inputs."""
        super().__init__("neato_follower_node")

        # Create a publisher that publishes to topic "/cmd_vel" controlling robot speed
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create a subscriber to "/scan," getting LIDAR data
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

    def on_scan(self, data):
        """
        HELLO
        """

    def run_loop(self):
        """
        HELLO
        """


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = NeatoFollower()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
