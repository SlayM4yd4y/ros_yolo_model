from launch import LaunchDescription
from launch_ros.actions import Node
#TODO: megcsinalni ugy hogy ez csak a tanitas inditasahoz kelljen paramerekkel, a card gen az siman run-os maradhat 
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
