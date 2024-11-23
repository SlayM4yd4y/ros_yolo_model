#include "teach.hpp"
#include "get_package_path.hpp"
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace fs = std::filesystem;

YOLOTrainer::YOLOTrainer() : Node("yolo_trainer") {
    RCLCPP_INFO(this->get_logger(), "YOLO Trainer node inicializalva");
}

void YOLOTrainer::train(const std::string& dataset_path, const std::string& model_output_path) {
    // Adatkészlet közvetlen elérése a letöltött és kicsomagolt adatokból
    std::string data_yaml = dataset_path + "/data.yaml";
    // Tanítás indítása
    start_training(data_yaml, model_output_path);
}

void YOLOTrainer::start_training(const std::string& data_yaml, const std::string& model_output_path) {
    RCLCPP_INFO(this->get_logger(), "Tanitas megkezdese...");
    // YOLOv5 modell betanítási parancs 16 batch-el, 100 epoch-al, elegnek kell lennie
    //std::string command = "python3 /home/ajr/ros2_ws/src/yolov5/train.py --img 640 --batch 16 --epochs 100 --data " + data_yaml + " --weights yolov5s.pt --cache --name " + model_output_path;
    // YOLOv5 modell fine-tuning parancs korábbi súlyokkal (best.pt) folytatva
    /*std::string command = "python3 /home/ajr/ros2_ws/src/yolov5/train.py --img 640 --batch 16 --epochs 20 --data " + data_yaml + 
                          " --weights /home/ajr/ros2_ws/src/ros_yolo_model/output_model_fine/weights/best.pt --cache --name " + model_output_path + " --hyp /home/ajr/ros2_ws/src/ros_yolo_model/dataset/hyp.yaml";*/;
    std::string command = "python3 " + package_path() + "/../yolov5/train.py --img 640 --batch 16 --epochs 50 --data " + data_yaml + " --weights " + package_path() + "/model/fifth_train[cards]/weights/best.pt --cache --name " + model_output_path + " --hyp " + "/merged_dataset/hyp.yaml";

    int result = std::system(command.c_str());
    if (result != 0) {RCLCPP_ERROR(this->get_logger(), "Nem sikerult a tanitas.");
    } else {RCLCPP_INFO(this->get_logger(), "Sikeres tanitas.");}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<YOLOTrainer>();
    // Paraméterek beolvasása
    if (argc < 3) {
        std::cerr << "Hasznalat: ros2 run ros_yolo_model teach_node <dataset_path> <model_output_path>" << std::endl;
        return 1;
    }
    std::string dataset_path = argv[1];  // A  adatok elérési útvonala (dataset)
    std::string model_output_path = argv[2];

    // Modell tanítása indítás
    node->train(dataset_path, model_output_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
