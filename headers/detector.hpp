#ifndef ROS_YOLO_MODEL_DETECTOR_HPP_
#define ROS_YOLO_MODEL_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class YOLODetector : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_pub_;    
public:
    YOLODetector();
    void detect_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif  // ROS_YOLO_MODEL_DETECTOR_HPP_
