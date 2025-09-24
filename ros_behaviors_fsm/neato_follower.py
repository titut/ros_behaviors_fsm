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

        # Follower variables
        self.Kp_angle = 0.6
        self.Kp_linear = 0.5
        self.desired_location = np.array([0, 0])

        # LIDAR scan data in x, y coordinates in Neato frame
        self.dist_x = []
        self.dist_y = []

    def on_scan(self, data):
        """
        Gets scan data. Process it into coordinates in Neato frame.

        NOTE: LIDAR Scan index 0 is facing forward. As index increases, scan
        rotates CCW.
        """
        # clear distance list
        self.dist_x.clear()
        self.dist_y.clear()

        # convert scan data to coordinates
        for i in range(360):
            self.dist_x.append(-sin(radians(i)) * data.ranges[i])
            self.dist_y.append(cos(radians(i)) * data.ranges[i])

        # get closest obstacles
        self.find_objects(np.array([self.dist_x, self.dist_y]).transpose())

    def find_objects(self, coords):
        """
        Gets numpy array of coordinates in Neato frame. Segment points using
        DBSCAN into distinct objects. Get objects based on size and distance
        to Neato. Return coordinates associated with object.
        """
        db_default = DBSCAN(eps=0.2, min_samples=3).fit(coords)

        obj_coords = [[[], []] for _ in range(7)]
        min_dist = 100
        min_moment = None

        for i in range(coords.shape[0]):
            if db_default.labels_[i] < 6 and db_default.labels_[i] != -1:
                obj_coords[db_default.labels_[i]][0].append(coords[i, 0])
                obj_coords[db_default.labels_[i]][1].append(coords[i, 1])

        for index, obj_coord in enumerate(obj_coords):
            obj = np.array(obj_coord).transpose()
            moment = self.calculate_moment(obj)
            dist = 100
            if moment is not None:
                dist = self.euclidian_dist(moment, [0, 0])
            if dist < min_dist:
                min_dist = dist
                min_moment = moment

        self.desired_location = min_moment

    def calculate_moment(self, coords):
        """
        Get coordinates of object. Return the moment of object.
        """
        x_total = 0
        y_total = 0
        if coords.shape[0] == 0 or coords.shape[0] > 50:
            return None
        for i in range(coords.shape[0]):
            x_total = x_total + coords[i, 0]
            y_total = y_total + coords[i, 1]

        return np.array([x_total / coords.shape[0], y_total / coords.shape[0]])

    def euclidian_dist(self, point1, point2):
        """
        Return euclidian distance between two points
        """
        return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def run_loop(self):
        """
        Moves towards desired location using proportional control
        """

        angle_err = (
            atan2(self.desired_location[1], self.desired_location[0]) - np.pi / 2
        )
        distance_err = self.euclidian_dist(self.desired_location, [0, 0]) - 0.5

        linear = Vector3(
            x=distance_err * self.Kp_linear if distance_err > 0.1 else 0.0,
            y=0.0,
            z=0.0,
        )
        angular = Vector3(
            x=0.0,
            y=0.0,
            z=(angle_err * self.Kp_angle if abs(angle_err) > 0.2 else 0.0),
        )

        twist = Twist(linear=linear, angular=angular)

        self.publisher.publish(twist)


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
