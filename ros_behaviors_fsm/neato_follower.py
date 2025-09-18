"""Follow a Neato based on LIDAR Scan"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Twist, Vector3
from sensor_msgs.msg import LaserScan

from math import radians, cos, sin
import matplotlib.pyplot as plt


class NeatoFollower(Node):
    """Neato following node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the node. No inputs."""
        super().__init__("neato_follower_node")

        # Create a publisher that publishes to topic "/cmd_vel" controlling robot speed
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Create a subscriber to "/scan," getting LIDAR data
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)

        self.dist_x = []
        self.dist_y = []

    def on_scan(self, data):
        """
        Gets scan data.

        NOTE: LIDAR Scan index 0 is facing forward. As index increases, scan
        rotates CCW.
        """
        # clear distance list
        self.dist_x.clear()
        self.dist_y.clear()

        for i in range(360):
            self.dist_x.append(-sin(radians(i)) * data.ranges[i])
            self.dist_y.append(cos(radians(i)) * data.ranges[i])

        plt.plot(self.dist_x, self.dist_y, linestyle="None", marker="o")
        plt.show()


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
