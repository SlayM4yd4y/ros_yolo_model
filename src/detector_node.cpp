#include "detector_node.hpp"
#include "get_package_path.hpp"
#include <sstream>
#include <cstdlib>
#include <future>

DetectorNode::DetectorNode() : Node("detector_node") {
    RCLCPP_INFO(this->get_logger(), "DetectorNode konstruktora elindult.");
    weights_path_ = declare_parameter("weights_path", package_path() + "/model/fifth_train[cards]/weights/best.pt");
    source_type_ = declare_parameter("source_type", "camera");
    video_path_ = declare_parameter("video_path", "path");
    image_path_ = declare_parameter("image_path", "path");
    camera_id_ = declare_parameter("camera_id", 0);
    conf_thres_ = declare_parameter("conf_thres", 0.25);
    iou_thres_ = declare_parameter("iou_thres", 0.45);
    save_results_ = declare_parameter("save_results", true);
    save_dir_ = declare_parameter("save_dir", "path");
    view_img_ = declare_parameter("view_img", true);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&DetectorNode::detectImageCallback, this, std::placeholders::_1));
    object_pub_ = this->create_publisher<std_msgs::msg::String>("detected_objects", 10);
    RCLCPP_INFO(this->get_logger(), "DetectorNode sikeresen inicializálva.");
}

void DetectorNode::run() {
    RCLCPP_INFO(this->get_logger(), "Run függvény elindult.");

    if (source_type_ == "video") {
        RCLCPP_INFO(this->get_logger(), "Video forrást választott.");
        detectVideo(video_path_);
    } else if (source_type_ == "image") {
        RCLCPP_INFO(this->get_logger(), "Image forrást választott.");
        detectImage(image_path_);
    } else if (source_type_ == "camera") {
        RCLCPP_INFO(this->get_logger(), "Kamera forrást választott.");
        //automatikusan meghivodik a callback amikor érkezik a kép az image topicon
    } else {
        RCLCPP_ERROR(this->get_logger(), "Érvénytelen source_type: %s", source_type_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Run függvény véget ért.");
}


void DetectorNode::executeDetectionCommand(const std::string& source) {
     RCLCPP_INFO(this->get_logger(), "executeDetectionCommand elkezdődött forrás: %s", source.c_str());

    std::stringstream command;
    command << "python3 " + package_path() + "/../yolov5/detect.py --weights " << weights_path_
            << " --source " << source
            << " --conf-thres " << conf_thres_
            << " --iou-thres " << iou_thres_;

    if (save_results_) { command << " --project " << save_dir_ << " --save-txt"; }
    if (view_img_) { command << " --view-img"; }

    std::string command_str = command.str();
    RCLCPP_INFO(this->get_logger(), "Parancs futtatása: %s", command_str.c_str());

    //külön szálon
    std::thread exec_thread([command_str]() {
        int result = std::system(command_str.c_str());
        if (result != 0) {
            std::cerr << "A szkript futása sikertelen, hiba történt: " << result << std::endl;
        }
    });
    exec_thread.join();  //varakozas amig bef. a szal
    RCLCPP_INFO(this->get_logger(), "Detektálás véget ért forrásnál: %s", source.c_str());
}

void DetectorNode::detectImage(const std::string& image_path) {
    executeDetectionCommand(image_path);

    auto result_msg = std::make_shared<std_msgs::msg::String>();
    result_msg->data = "Képdetektálás befejezve.";
    object_pub_->publish(*result_msg);
    RCLCPP_INFO(this->get_logger(), "Eredmény üzenet publikálva a /detected_objects topic-ra.");
    
}


void DetectorNode::detectImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::string image_path = "/tmp/input_image.jpg";
    cv::imwrite(image_path, frame);
    executeDetectionCommand(image_path);
    auto result_msg = std::make_shared<std_msgs::msg::String>();
    result_msg->data = "Élő képdetektálás befejezve.";
    object_pub_->publish(*result_msg);
    RCLCPP_INFO(this->get_logger(), "Eredmény üzenet publikálva a /detected_objects topic-ra.");
    
}


/*void DetectorNode::detectLiveCamera(int camera_id) {
    executeDetectionCommand(std::to_string(camera_id));

    auto result_msg = std::make_shared<std_msgs::msg::String>();
    result_msg->data = "Élő kamera detektálás befejezve.";
    object_pub_->publish(*result_msg);
    RCLCPP_INFO(this->get_logger(), "Eredmény üzenet publikálva a /detected_objects topic-ra.");
    
}*/ //felesleges

void DetectorNode::detectVideo(const std::string& video_path) {
    executeDetectionCommand(video_path);
    auto result_msg = std::make_shared<std_msgs::msg::String>();
    result_msg->data = "Videó detektálás befejezve.";
    object_pub_->publish(*result_msg);
    RCLCPP_INFO(this->get_logger(), "Eredmény üzenet publikálva a /detected_objects topic-ra.");
    
}

int main(int argc, char **argv) {
    std::cout << "Program elindult" << std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();
    RCLCPP_INFO(node->get_logger(), "DetectorNode létrehozva és futtatásra kész.");
    node->run();
    rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}