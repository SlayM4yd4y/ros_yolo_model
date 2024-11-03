# ROS YOLO MODEL

## Diagram
graph LR;

video_publisher([ /video_publisher_node]):::red --> image[ /image<br/>sensor_msgs/Image]:::light
detector([ /detector_node]):::red --> detected_objects[ /detected_objects<br/>std_msgs/String]:::light
detector --> image

classDef light fill:#34aec5,stroke:#152742,stroke-width:2px,color:#152742  
classDef dark fill:#152742,stroke:#34aec5,stroke-width:2px,color:#34aec5
classDef white fill:#ffffff,stroke:#152742,stroke-width:2px,color:#152742
classDef red fill:#ef4638,stroke:#152742,stroke-width:2px,color:#fff
