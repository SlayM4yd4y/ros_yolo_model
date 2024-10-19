#ifndef ROS_YOLO_MODEL_TEACH_HPP_
#define ROS_YOLO_MODEL_TEACH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class YOLOTrainer : public rclcpp::Node
{
public:
    YOLOTrainer();
    void train_model(const std::string& train_data_path, const std::string& val_data_path, const std::string& output_model_path);
};

#endif  // ROS_YOLO_MODEL_TEACH_HPP_
