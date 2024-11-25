#include "video_publisher_node.hpp"

VideoPublisherNode::VideoPublisherNode() : Node("video_publisher_node") {
    bool use_camera = declare_parameter("use_camera", true);
    if (!use_camera) {
        RCLCPP_INFO(this->get_logger(), "A kamera használata letiltva. A node leáll.");
        rclcpp::shutdown();
        return;
    }
    declare_parameter("camera_id", 0);
    declare_parameter("camera_ip", "");
    get_parameter("camera_id", camera_id_);
    get_parameter("camera_ip", camera_ip_);
    fps_ = declare_parameter("fps", 30);  

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image", 10);

    if (!camera_ip_.empty()) {
        RCLCPP_INFO(this->get_logger(), "IP-alapú kameraforrás használata: %s", camera_ip_.c_str());
        if (!cap_.open(camera_ip_)) {
            RCLCPP_ERROR(this->get_logger(), "Nem sikerült megnyitni az IP-alapú kamerát: %s", camera_ip_.c_str());
            rclcpp::shutdown();
            return;
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Helyi kamera használata, ID: %d", camera_id_);
        if (!cap_.open(camera_id_)) {
            RCLCPP_ERROR(this->get_logger(), "Nem sikerült megnyitni a helyi kamerát, ID: %d", camera_id_);
            rclcpp::shutdown();
            return;
        }
    }
    timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / fps_),
        std::bind(&VideoPublisherNode::publishFrame, this));
}

void VideoPublisherNode::publishFrame() {
    cv::Mat frame;
    cap_ >> frame; 
    if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "Üres képkocka a kamerától.");
        return;
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_pub_->publish(*msg);
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}