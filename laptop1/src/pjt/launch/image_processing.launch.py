import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pjt",
                executable="detect_panel_node",
                output="screen",
            ),
            # rqt_image_view 실행
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rqt_image_view",
                    "rqt_image_view",
                    "--ros-args",
                    "-r",
                    "/image:=/camera/color/detection_image",
                ],
                output="screen",
            ),
        ]
    )
