from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    source_type = LaunchConfiguration("source_type").perform(context)
    save_results = LaunchConfiguration("save_results").perform(context).lower() == "true"
    save_dir = LaunchConfiguration("save_dir").perform(context)
    nodes = []
    nodes.append(
        Node(
            package="ros_yolo_model",
            executable="detector_node",
            name="detector_node",
            parameters=[
                {"weights_path": LaunchConfiguration("weights_path")},
                {"source_type": source_type},
                {"video_path": LaunchConfiguration("video_path")},
                {"image_path": LaunchConfiguration("image_path")},
                {"camera_id": LaunchConfiguration("camera_id")},
                {"camera_ip": LaunchConfiguration("camera_ip")},
                {"save_results": save_results},
                {"save_dir": save_dir},
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
                parameters=[{"use_camera": True},
                            {"camera_ip": LaunchConfiguration("camera_ip")},
                            {"camera_id": LaunchConfiguration("camera_id")}
                ],
                output="screen"
            )
        )
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("weights_path", default_value="/home/ajr/ros2_ws/src/ros_yolo_model/model/fifth_train[cards]/weights/best.pt", description="Modell súlyok elérési útvonala"),
        DeclareLaunchArgument("source_type", default_value="camera", description="Forrás típusa: camera, video, vagy image"),
        DeclareLaunchArgument("video_path", default_value="/path/to/video.mp4", description="Videó elérési útvonala"),
        DeclareLaunchArgument("image_path", default_value="/path/to/image.jpg", description="Kép elérési útvonala"),
        DeclareLaunchArgument("camera_id", default_value="0", description="Camera ID"),
        DeclareLaunchArgument("camera_ip", default_value="http://default", description="Webkamera IP címe(http://ip:port)"),
        DeclareLaunchArgument("save_results", default_value="true", description="Eredmények mentése (true/false)"),
        DeclareLaunchArgument("save_dir", default_value="/home/ajr/ros2_ws/src/ros_yolo_model/det_results", description="Eredmények mentési mappája"),
        DeclareLaunchArgument("view_img", default_value="true", description="Valós idejű detektálás megjelenítése"),
        OpaqueFunction(function=launch_setup)
    ])
