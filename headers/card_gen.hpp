#ifndef ROS_YOLO_MODEL_CARD_GEN_HPP_
#define ROS_YOLO_MODEL_CARD_GEN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class CardGenerator : public rclcpp::Node
{
private:
    static int counter;
public:
    CardGenerator();
    void generate_card(const std::string& output_dir, int count);
};

#endif  // ROS_YOLO_MODEL_CARD_GEN_HPP_
