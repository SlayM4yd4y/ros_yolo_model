#ifndef ROS_YOLO_MODEL_CARD_GEN_HPP_
#define ROS_YOLO_MODEL_CARD_GEN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/freetype.hpp>
#include <filesystem>
#include <unistd.h>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <iostream>
#include <vector>

class CardGenerator : public rclcpp::Node
{
private:
    //static int counter;
    std::vector<std::string> names;
    std::vector<std::string> nCodes;
    std::vector<std::string> ids;
    std::vector<std::string> images;
    std::vector<std::string> barcodes;
    std::vector<std::string> cardIds;
    std::vector<std::string> cardTemplates;

public:
    CardGenerator();
    void generate_card(const std::string& output_dir);
};

std::string package_path();

#endif  // ROS_YOLO_MODEL_CARD_GEN_HPP_
