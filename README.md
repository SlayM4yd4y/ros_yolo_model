# `ROS YOLO MODEL` ROS2 Package
A ```ROS 2``` package, written in ```C++```, leveraging ```YOLOv5``` for object detection in various input sources (camera, video, image). This project integrates deep learning-based detection into the ROS 2 framework, providing flexibility for robotics applications. Developed and tested in a ```WSL``` (Windows Subsystem for Linux) environment.

### Features
* **Live Camera Detection**: Detection with camera streams.
* **Video & Image Detection**: Process pre-recorded videos or individual images.
* **Customizable Classes**: Detects 22 object classes, including PASCAL VOC 2012 dataset objects and custom classes like university student and employee ID cards.
* **ROS Topics Integration**: Publishes detection results on `/detected_objects` topic.
The implementation is designed to run under ROS 2 Humble.
### Clone the packages
>It is assumed that the workspace is `~/ros2_ws/`.
``` 
cd ~/ros2_ws/src
```
```
git clone https://github.com/SlayM4yd4y/ros_yolo_model
```
```
git clone https://github.com/ultralytics/yolov5  
cd yolov5
pip install -r requirements.txt  
```
## Build this ROS 2 package
>It is assumed that the workspace is still `~/ros2_ws/`.
```
cd ~/ros2_ws
```
```
colcon build --packages-select ros_yolo_model --symlink-install
```
## Run this ROS 2 package
<details>
<summary> Don't forget to source before ROS commands.</summary>
source ~/ros2_ws/install/setup.bash
</details>
<div align="center"><h3>For more details visit the /wiki of this package.</h3></div>

## Diagram
``` mermaid
graph LR;

video_publisher([ /video_publisher_node]):::red --> image[ /image<br/>sensor_msgs/Image]:::light
image --> detector([ /detector_node]):::red
detector --> detected_objects[ /detected_objects<br/>std_msgs/String]:::light

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
```

