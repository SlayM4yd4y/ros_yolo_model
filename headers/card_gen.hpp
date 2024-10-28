#ifndef ROS_YOLO_MODEL_CARD_GEN_HPP_
#define ROS_YOLO_MODEL_CARD_GEN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <filesystem>
#include <unistd.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>

class CardGenerator : public rclcpp::Node
{
private:
    //static int counter;
public:
    CardGenerator();
    void generate_card(const std::string& output_dir, int count);
};

std::string package_path();

#endif  // ROS_YOLO_MODEL_CARD_GEN_HPP_
