from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_behaviors_fsm",
                executable="fsm",
                name="fsm_node",
                output="screen",
            ),
            Node(
                package="ros_behaviors_fsm",
                executable="drive_straight",
                name="drive_straight_node",
                output="screen",
            ),
            Node(
                package="ros_behaviors_fsm",
                executable="drive_star",
                name="drive_star_node",
                output="screen",
            ),
        ]
    )
