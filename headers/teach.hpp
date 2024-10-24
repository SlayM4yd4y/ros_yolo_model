#ifndef ROS_YOLO_MODEL_TEACH_HPP_
#define ROS_YOLO_MODEL_TEACH_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>

class YOLOTrainer : public rclcpp::Node {
private:
    //bool download_dataset(const std::string& dataset_url, const std::string& output_dir); CSAK API-s tanításnál kell
    void start_training(const std::string& data_yaml, const std::string& model_output_path);
protected:
    //std::string roboflow_api_key; mivel nem api-s tanítás volt, ezért nem kell, de van ilyen lehetoseg is
public:
    YOLOTrainer();
    void train(const std::string& dataset_url, const std::string& model_output_path);

};

#endif  
