#ifndef DETECTOR_NODE_HPP
#define DETECTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode();
    void run();
private:
    void detectImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void executeDetectionCommand(const std::string& source);
    void detectLiveCamera(int camera_id);
    void detectImage(const std::string& image_path);
    void detectVideo(const std::string& video_path);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr object_pub_;
    std::string weights_path_;
    std::string data_path_;
    std::string source_type_;  
    std::string video_path_;   
    std::string image_path_;
    std::string save_dir_;    
    int camera_id_;             
    float conf_thres_;
    float iou_thres_;
    bool view_img_;
};

#endif  // DETECTOR_NODE_HPP