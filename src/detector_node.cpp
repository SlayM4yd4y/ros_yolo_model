#include "detector_node.hpp"
#include <sstream>
#include <cstdlib>

DetectorNode::DetectorNode() : Node("detector_node") {
    // Paraméterek inicializálása
    weights_path_ = declare_parameter("weights_path", "/home/ajr/ros2_ws/src/ros_yolo_model/model/second_train/weights/best.pt");
    source_type_ = declare_parameter("source_type", "camera");
    video_path_ = declare_parameter("video_path", "/path/to/video.mp4");
    image_path_ = declare_parameter("image_path", "/path/to/image.jpg");
    camera_id_ = declare_parameter("camera_id", 0);
    conf_thres_ = declare_parameter("conf_thres", 0.25);
    iou_thres_ = declare_parameter("iou_thres", 0.45);
    save_dir_ = declare_parameter("save_dir", "/path/to/output");
    view_img_ = declare_parameter("view_img", true);

    // Kép előfizetés és eredmény közzététel
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&DetectorNode::detectImageCallback, this, std::placeholders::_1));
    object_pub_ = this->create_publisher<std_msgs::msg::String>("detected_objects", 10);
}


void DetectorNode::executeDetectionCommand(const std::string& source) {
    std::stringstream command;
    command << "python3 /home/ajr/ros2_ws/src/yolov5/detect.py --weights " << weights_path_
            << " --source " << source
            << " --conf-thres " << conf_thres_
            << " --iou-thres " << iou_thres_
            << " --project " << save_dir_
            << " --save-txt";
    
    if (view_img_) {command << " --view-img";}
    int result = std::system(command.str().c_str());
    if (result != 0) {
        RCLCPP_ERROR(this->get_logger(), "Detektálási parancs sikertelen forrásnál: %s", source.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "Detektálás befejezve forrásnál: %s", source.c_str());
    }
}

void DetectorNode::detectLiveCamera(int camera_id) {
    executeDetectionCommand(std::to_string(camera_id));

    auto result_msg = std::make_shared<std_msgs::msg::String>();
    result_msg->data = "Élő kamera detektálás befejezve.";
    object_pub_->publish(*result_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();

    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}