from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_yolo_model',
            executable='teach_node',
            name='teach'
        ),
        Node(
            package='ros_yolo_model',
            executable='card_gen_node',
            name='card_gen'
        )
    ])
