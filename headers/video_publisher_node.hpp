#ifndef VIDEO_PUBLISHER_NODE_HPP
#define VIDEO_PUBLISHER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoPublisherNode : public rclcpp::Node {
public:
    VideoPublisherNode();
    bool use_camera;
private:
    void publishFrame();
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    int camera_id_;
    int fps_;
};

#endif  // VIDEO_PUBLISHER_NODE_HPP
