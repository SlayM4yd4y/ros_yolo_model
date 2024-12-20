#include "detector_node.hpp"
#include "get_package_path.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <future>


namespace fs = std::filesystem;

DetectorNode::DetectorNode() : Node("detector_node"), stop_processing_(false) {
    RCLCPP_INFO(this->get_logger(), "DetectorNode konstruktora elindult.");
    weights_path_ = declare_parameter("weights_path", package_path() + "/model/fifth_train[cards]/weights/best.pt");
    source_type_ = declare_parameter("source_type", "camera");
    video_path_ = declare_parameter("video_path", "path");
    image_path_ = declare_parameter("image_path", "path");
    camera_id_ = declare_parameter("camera_id", 0);
    camera_ip_ = declare_parameter("camera_ip", "http");
    conf_thres_ = declare_parameter("conf_thres", 0.25);
    iou_thres_ = declare_parameter("iou_thres", 0.45);
    save_results_ = declare_parameter("save_results", true);
    save_dir_ = declare_parameter("save_dir", "path");
    view_img_ = declare_parameter("view_img", true);

    auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", qos, std::bind(&DetectorNode::detectImageCallback, this, std::placeholders::_1));
    object_pub_ = this->create_publisher<std_msgs::msg::String>("/detected_objects", rclcpp::QoS(rclcpp::SystemDefaultsQoS()));
    processing_thread_ = std::thread(&DetectorNode::processingLoop, this);
    RCLCPP_INFO(this->get_logger(), "DetectorNode sikeresen inicializálva.");
}

DetectorNode::~DetectorNode() {
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        stop_processing_ = true;
    }
    queue_condition_.notify_one();
    if(processing_thread_.joinable()) {processing_thread_.join();}
    RCLCPP_INFO(this->get_logger(), "DetectorNode destruktora sikeresen lefutott.");
}

void DetectorNode::processingLoop() {
    while (true) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_condition_.wait(lock, [this]() { return !frame_queue_.empty() || stop_processing_; });
            if (stop_processing_) break;
            frame = frame_queue_.front();
            frame_queue_.pop();
        }
        if (!frame.empty()) {
            std::string image_path = "/tmp/input_image.jpg";
            cv::imwrite(image_path, frame);
            executeDetectionCommand(image_path);
        }
    }
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
    } else {
        RCLCPP_ERROR(this->get_logger(), "Érvénytelen source_type: %s", source_type_.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Run függvény véget ért.");
}

std::string DetectorNode::getLatestExpFolder(const std::string& base_path) {
    std::string latest_folder;
    std::time_t latest_time = 0;
    for (const auto& entry : fs::directory_iterator(base_path)) {
        if (entry.is_directory() && entry.path().filename().string().rfind("exp", 0) == 0) {
            auto mod_time = fs::last_write_time(entry);
            auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                mod_time - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
            auto sys_time = std::chrono::system_clock::to_time_t(sctp);
            if (sys_time > latest_time) {
                latest_time = sys_time;
                latest_folder = entry.path().string();
            }
        }
    }
    return latest_folder;
}

void DetectorNode::executeDetectionCommand(const std::string& source) {
    RCLCPP_INFO(this->get_logger(), "executeDetectionCommand elkezdődött forrás: %s", source.c_str());
    std::stringstream command;
    command << "python3 " + package_path() + "/../yolov5/detect.py --weights " << weights_path_
            << " --source " << source
            << " --conf-thres " << conf_thres_
            << " --iou-thres " << iou_thres_;
    if(save_results_) {command << " --project " << save_dir_ << " --save-txt";}
    if (view_img_) {command << " --view-img";}
    //külön szálon
    std::thread exec_thread([this, command_str = command.str()]() {
        int result = std::system(command_str.c_str());
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "A szkript futása sikertelen, hiba történt: %d", result);
        }
    });

    exec_thread.join();
    if(save_results_){
        auto detected_objects = parseDetectionResults(getLatestExpFolder(save_dir_) + "/labels");
        for (const auto& obj : detected_objects) {
            auto result_msg = std::make_shared<std_msgs::msg::String>();
            result_msg->data = obj;
            object_pub_->publish(*result_msg);
            RCLCPP_INFO(this->get_logger(), "Detektált objektum: %s", obj.c_str());}
    } else {
        RCLCPP_INFO(this->get_logger(), "Nem mentett eredmény nem publikálható.");
        auto result_msg = std::make_shared<std_msgs::msg::String>();
        result_msg->data = "<<Nem mentett eredmény nem publikálható.>>";
        object_pub_->publish(*result_msg);
    }
}

std::vector<std::string> DetectorNode::parseDetectionResults(const std::string& results_dir) {
    
    std::vector<std::string> detected_objects;
    std::map<int, std::string> class_map = {
        {0, "Aeroplane"}, {1, "Bicycle"}, {2, "Bird"}, {3, "Boat"}, {4, "Bottle"},
        {5, "Bus"}, {6, "Car"}, {7, "Cat"}, {8, "Chair"}, {9, "Cow"},
        {10, "Dining Table"}, {11, "Dog"}, {12, "Horse"}, {13, "Motorbike"}, {14, "Person"},
        {15, "Potted Plant"}, {16, "Sheep"}, {17, "Sofa"}, {18, "Train"}, {19, "TV Monitor"},
        {20, "Alkalmazotti Kártya"}, {21, "Hallgatói Kártya"}
    };
     for (const auto& entry : fs::directory_iterator(results_dir)) {
        if (entry.path().extension() == ".txt") {
            std::ifstream file(entry.path());
            if (!file.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Nem sikerült megnyitni a fájlt: %s", entry.path().c_str());
                continue;
            }

            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                int class_id;
                float x, y, w, h;
                if (iss >> class_id >> x >> y >> w >> h) {
                    std::string object_name = (class_map.find(class_id) != class_map.end()) ? class_map[class_id] : "Ismeretlen osztály";
                    detected_objects.push_back(object_name);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Nem megfelelő formátumú sor: %s", line.c_str());
                }
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Fájl kihagyva (nem .txt): %s", entry.path().c_str());
        }
    }
    if (detected_objects.empty()) {detected_objects.push_back("Nem található objektum.");}
    return detected_objects;
}

void DetectorNode::detectImage(const std::string& image_path) {
    executeDetectionCommand(image_path);
}

void DetectorNode::detectImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    static std::mutex mtx;
    if (!mtx.try_lock()) {
        RCLCPP_WARN(this->get_logger(), "Már folyamatban van egy feldolgozás, kihagyom ezt a képkockát.");
        return;
    }
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    std::thread([this, frame, mutex_ptr = &mtx]() {
        std::string image_path = "/tmp/input_image.jpg";
        cv::imwrite(image_path, frame);
        executeDetectionCommand(image_path);
        mutex_ptr->unlock(); 
    }).detach();
}

void DetectorNode::detectVideo(const std::string& video_path) {
    executeDetectionCommand(video_path);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectorNode>();
    node->run();
    node->get_parameter("source_type").as_string() == "camera" ? rclcpp::spin(node) : rclcpp::spin_some(node);
    rclcpp::shutdown();
    return 0;
}