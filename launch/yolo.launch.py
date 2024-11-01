from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_yolo_model',
            executable='teach_node',
            name='teach'
        )
    ])
