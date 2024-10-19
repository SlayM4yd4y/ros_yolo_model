#ifndef ROS_YOLO_MODEL_CARD_GEN_HPP_
#define ROS_YOLO_MODEL_CARD_GEN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class CardGenerator : public rclcpp::Node
{
public:
    CardGenerator();
    void generate_card(const std::string& output_dir);
};

#endif  // ROS_YOLO_MODEL_CARD_GEN_HPP_
