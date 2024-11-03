# `ROS YOLO MODEL` ROS2 Package

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

