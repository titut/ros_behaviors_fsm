import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class DriveStraightNode(Node):
    """This class drives the Neato forward"""

    def __init__(self):
        super().__init__("drive_straight")
        self.create_timer(0.1, self.run_loop)
        self.create_timer(0.1, self.testfunc)

        self.create_subscription(String, "state", self.process_msg, 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.is_drive = False
        self.i = 0

    def testfunc(self):
        if self.i >= 50:
            self.is_drive = not self.is_drive
            self.i = 0
        self.i += 1

    def process_msg(self, msg):
        if msg == "forward":
            self.is_drive = True
        else:
            self.is_drive = False

    def run_loop(self):
        if self.is_drive is False:
            self.drive(0.0, 0.0)
        else:
            self.drive(0.2, 0.0)

    def drive(self, linear, angular):
        """Drive with the specified linear and angular velocity.

        Args:
            linear (_type_): the linear velocity in m/s
            angular (_type_): the angular velocity in radians/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DriveStraightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
