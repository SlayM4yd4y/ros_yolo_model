#include "detector.hpp"
#include <std_msgs/msg/string.hpp>

YOLODetector::YOLODetector() : Node("yolo_detector") {
    RCLCPP_INFO(this->get_logger(), "YOLO Detector Node elindult.");
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image", 10, std::bind(&YOLODetector::detect_callback, this, std::placeholders::_1));

    object_pub_ = this->create_publisher<std_msgs::msg::String>("object_detection", 10);
}

void YOLODetector::detect_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat frame = cv_ptr->image;

    // YOLO modell használata a detektáláshoz (TODO: itt kéne a detektálást implementálni)
    std_msgs::msg::String result_msg;
    result_msg.data = "Object detected"; // Ezt cseréld ki a valódi detektálás eredményére

    // Publikálás az object_detection topicra
    object_pub_->publish(result_msg);

    // Eredmények megjelenítése OpenCV ablakban
    cv::imshow("YOLO Detection", frame);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto detector = std::make_shared<YOLODetector>();

    rclcpp::spin(detector);
    rclcpp::shutdown();
    return 0;
}
