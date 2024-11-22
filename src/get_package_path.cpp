#include "get_package_path.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>



std::string package_path() {
    std::string package_name = "ros_yolo_model";
    std::string package_path_shared = ament_index_cpp::get_package_share_directory(package_name);
    std::string package_path = package_path_shared + "/../../../../src/ros_yolo_model";
    std::cout << "The package path is: " << package_path << std::endl;
    return package_path;
}