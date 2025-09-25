"""
This file contains the node to drive the Neato in a star shape.
"""

from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class DriveStar(Node):
    """
    ROS2 Node that drives the Neato in a star shape.
    """

    def __init__(self):
        super().__init__("drive_star_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.state_sub = self.create_subscription(
            String, "state", self.process_state, 10
        )
        self.star_pub = self.create_publisher(Bool, "star_status", 10)
        self.create_timer(0.1, self.run_loop)
        self.turns_executed = 0
        self.executing_turn = False
        self.side_length = 1.5  # the length in meters of a square side
        self.time_per_side = 5.0  # duration in seconds to drive the square side
        self.time_per_turn = 3.0  # duration in seconds to turn 90 degrees
        # start_time_of_segment indicates when a particular part of the square was
        # started (e.g., a straight segment or a turn)
        self.start_time_of_segment = None

        self.state = None
        self.star_status = Bool()

    def process_state(self, msg):
        self.state = msg.data

    def run_loop(self):
        """In the run_loop we are essentially implementing what's known as a finite-state
        machine.  That is, our robot code is in a particular state (in this case defined
        by whether or not we are turning and how many sides we've traversed thus far.
        """
        if self.state == "Star":
            if self.start_time_of_segment is None:
                self.start_time_of_segment = self.get_clock().now()
            msg = Twist()
            if self.executing_turn:
                segment_duration = self.time_per_turn
            else:
                segment_duration = self.time_per_side

            if (
                self.get_clock().now() - self.start_time_of_segment
                > rclpy.time.Duration(seconds=segment_duration)
            ):
                if self.executing_turn:
                    self.turns_executed += 1
                # toggle the executing_turn Boolean (turn to not turn or vice versa)
                self.executing_turn = not self.executing_turn
                self.start_time_of_segment = None
                print(self.executing_turn, self.turns_executed)
                # transition to next segment, don't change msg so we execute a stop
            else:
                if self.executing_turn:
                    # we are trying to turn 0.8 pi radians in a particular amount of time
                    # from this we can get the angular velocity
                    msg.angular.z = -(0.8 * pi) / segment_duration
                else:
                    msg.linear.x = self.side_length / segment_duration
            self.vel_pub.publish(msg)

            if self.turns_executed >= 5:
                self.star_status.data = True
            else:
                self.star_status.data = False

            self.star_pub.publish(self.star_status)


def main(args=None):
    rclpy.init(args=args)
    node = DriveStar()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
