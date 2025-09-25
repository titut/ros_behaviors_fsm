"""Finite State Machine, controlling all Nodes"""

from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class FiniteStateMachine(Node):
    """Finite state machine node, which inherits from the rclpy Node class."""

    def __init__(self):
        """Initializes the node. No inputs."""
        super().__init__("fsm_node")

        # Create a publisher that publishes to topic "state" controlling robot state
        self.publisher = self.create_publisher(String, "/state", 10)

        # Create a subscriber to "/scan_status" and "/star_status", getting other Node's status
        self.scan_sub = self.create_subscription(
            Bool, "/scan_status", self.scan_msg, 10
        )
        self.star_sub = self.create_subscription(Bool, "star_status", self.star_msg, 10)

        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Defaults to driving straight
        self.state = "Straight"
        self.switch_to_star = False
        self.start_time = self.get_clock().now()
        self.scan_status = False
        self.star_status = False

    def scan_msg(self, msg):
        """
        Processes /scan_status message
        """
        self.scan_status = bool(msg.data)

    def star_msg(self, msg):
        """
        Processes /star_status message
        """
        self.star_status = bool(msg.data)

    def run_loop(self):
        """
        Switches from Straight to Star to Follow depending on conditions
        """
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9

        if elapsed_time >= 5.0 and not self.switch_to_star:
            self.state = "Stop"
            sleep(2)
            self.state = "Star"
            self.switch_to_star = True

        if self.scan_status and self.star_status:
            self.state = "Follow"

        msg = String()
        msg.data = self.state
        self.publisher.publish(msg)


def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = FiniteStateMachine()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
