from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    source_type = LaunchConfiguration("source_type").perform(context)
    nodes = []
    nodes.append(
        Node(
            package="ros_yolo_model",
            executable="detector_node",
            name="detector_node",
            parameters=[
                {"weights_path": "/home/ajr/ros2_ws/src/ros_yolo_model/model/second_train/weights/best.pt"},
                {"source_type": source_type},
                {"video_path": LaunchConfiguration("video_path")},
                {"image_path": LaunchConfiguration("image_path")},
                {"camera_id": LaunchConfiguration("camera_id")},
                {"save_dir": "/home/ajr/ros2_ws/src/ros_yolo_model/det_results"},
                {"view_img": LaunchConfiguration("view_img")}
            ],
            output="screen"
        )
    )
    if source_type == "camera":
        nodes.append(
            Node(
                package="ros_yolo_model",
                executable="video_publisher_node",
                name="video_publisher_node",
                parameters=[{"use_camera": True}],
                output="screen"
            )
        )
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("source_type", default_value="camera", description="Forrás típusa: camera, video, vagy image"),
        DeclareLaunchArgument("video_path", default_value="/path/to/video.mp4", description="Videó fájl elérési útvonala"),
        DeclareLaunchArgument("image_path", default_value="/path/to/image.jpg", description="Kép fájl elérési útvonala"),
        DeclareLaunchArgument("camera_id", default_value="0", description="Camera ID"),
        DeclareLaunchArgument("view_img", default_value="false", description="Valós idejű detektálás megjelenítése"),
        OpaqueFunction(function=launch_setup)
    ])
