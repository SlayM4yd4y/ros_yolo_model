#include "card_augmenter_node.hpp"
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <random>

namespace fs = std::filesystem;

CardAugmenterNode::CardAugmenterNode(const std::string& cards_dir, const std::string& output_dir)
    : Node("card_augmenter_node"), rng_(std::random_device{}()), output_dir_(output_dir) {
    RCLCPP_INFO(this->get_logger(), "CardAugmenterNode konstruktor kezdete");
    setCardsDir(cards_dir);
    RCLCPP_INFO(this->get_logger(), "Kártyák betöltve a mappából: %s", cards_dir.c_str());
    std::string backgrounds_dir = "/home/ajr/ros2_ws/src/ros_yolo_model/img/background_samples";  
    for (const auto& entry : fs::directory_iterator(backgrounds_dir)) {
        background_images_paths_.emplace_back(entry.path().string());
        if (background_images_paths_.size() >= 20) break;
    }
    if (background_images_paths_.empty() || card_images_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Üres kártya- vagy háttérkép-mappa.");
        rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "CardAugmenterNode konstruktor vége");
}
// Getterek
inline const std::vector<cv::Mat>& CardAugmenterNode::getCardImages() const { return card_images_; }
inline const std::vector<std::string>& CardAugmenterNode::getBackgroundImagesPaths() const { return background_images_paths_; }
inline std::string CardAugmenterNode::getOutputDir() const { return output_dir_; }
// Setterek
inline void CardAugmenterNode::setCardsDir(const std::string& cards_dir) {
    card_images_.clear();
    RCLCPP_INFO(this->get_logger(), "Kártyák betöltése mappából: %s", cards_dir.c_str());
    for (const auto& entry : fs::directory_iterator(cards_dir)) {
        auto card_image = loadCardImage(entry.path().string());
        if (!card_image.empty()) {
            // 4csatornás kép konvertálása 3csatornásra
            if (card_image.channels() == 3) {
                cv::cvtColor(card_image, card_image, cv::COLOR_BGR2BGRA);
                RCLCPP_INFO(this->get_logger(), "Kártyakép konvertálva RGBA formátumba: %s", entry.path().c_str());
            }
            card_images_.emplace_back(card_image);
            RCLCPP_INFO(this->get_logger(), "Kép betöltve: %s", entry.path().c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Hiba a kártyakép betöltésében: %s", entry.path().c_str());
        }
    }
    RCLCPP_INFO(this->get_logger(), "Kártyák betöltése befejezve, összesen %lu kép található.", card_images_.size());
}

inline void CardAugmenterNode::setOutputDir(const std::string& output_dir) {
    if (!output_dir.empty()) {
        output_dir_ = output_dir;
    } else {
        RCLCPP_WARN(this->get_logger(), "Az output mappa elérési útja nem lehet üres.");
    }
}
inline cv::Mat CardAugmenterNode::loadCardImage(const std::string& image_path) {
    return cv::imread(image_path, cv::IMREAD_UNCHANGED);
}
inline cv::Mat CardAugmenterNode::loadRandomBackground() {
    std::uniform_int_distribution<int> dist(0, background_images_paths_.size() - 1);
    std::string background_path = background_images_paths_[dist(rng_)];
    RCLCPP_INFO(this->get_logger(), "Háttérkép betöltése innen: %s", background_path.c_str());
    return cv::imread(background_path);
}

cv::Mat CardAugmenterNode::augmentCard(const cv::Mat& card, const cv::Mat& background) {
    if (card.empty() || background.empty()) {
        RCLCPP_ERROR(this->get_logger(), "A kártya vagy a háttér üres kép!");
        return cv::Mat();
    }
    cv::Mat augmented_background = background.clone();
    // kártya méretének skálázása a háttérhez képest
    double scale_factor = std::min(
        static_cast<double>(background.cols) * 0.2 / card.cols,  
        static_cast<double>(background.rows) * 0.2 / card.rows
    );

    cv::Mat resized_card;
    cv::resize(card, resized_card, cv::Size(), scale_factor, scale_factor);
    // Elforgatas + uj keret merete
    int max_angle = 30;
    double angle = std::uniform_real_distribution<double>(-max_angle, max_angle)(rng_);
    double radians = angle * CV_PI / 180.0;
    int new_width = std::abs(resized_card.cols * std::cos(radians)) + std::abs(resized_card.rows * std::sin(radians));
    int new_height = std::abs(resized_card.cols * std::sin(radians)) + std::abs(resized_card.rows * std::cos(radians));
    // uj keret létrehozása és elforgatás
    cv::Mat larger_card(new_height, new_width, resized_card.type(), cv::Scalar(0, 0, 0, 0));
    cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(resized_card.cols / 2.0, resized_card.rows / 2.0), angle, 1.0);
    rotation_matrix.at<double>(0, 2) += (new_width - resized_card.cols) / 2.0;
    rotation_matrix.at<double>(1, 2) += (new_height - resized_card.rows) / 2.0;
    cv::warpAffine(resized_card, larger_card, rotation_matrix, larger_card.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    if (larger_card.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Az elforgatott kártya üres!");
        return augmented_background;
    }
    // hatterre helyezes
    cv::Point position(
        std::uniform_int_distribution<int>(0, background.cols - larger_card.cols)(rng_),
        std::uniform_int_distribution<int>(0, background.rows - larger_card.rows)(rng_)
    );

    cv::Mat roi = augmented_background(cv::Rect(position.x, position.y, larger_card.cols, larger_card.rows));

    for (int y = 0; y < larger_card.rows; ++y) {
        for (int x = 0; x < larger_card.cols; ++x) {
            cv::Vec4b pixel = larger_card.at<cv::Vec4b>(y, x);
            if (pixel[3] > 0) {
                roi.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel[0], pixel[1], pixel[2]);
            }
        }
    }
    return augmented_background;
}
inline void CardAugmenterNode::saveGeneratedImage(const cv::Mat& image, int index) {
    if (image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Az elmenteni kívánt kép üres, nem lehet menteni.");
        return;
    }
    std::string output_path = output_dir_ + "/generated_image__" + std::to_string(index) + ".jpg";
    cv::imwrite(output_path, image);
    RCLCPP_INFO(this->get_logger(), "Kép elmentve: %s", output_path.c_str());
}
void CardAugmenterNode::processAllCards() {
    RCLCPP_INFO(this->get_logger(), "processAllCards elkezdődött.");
    for (size_t i = 0; i < card_images_.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Háttér betöltése...");
        cv::Mat background = loadRandomBackground();
        if (background.empty()) {
            RCLCPP_ERROR(this->get_logger(), "A háttérkép betöltése sikertelen.");
            continue;
        }
        RCLCPP_INFO(this->get_logger(), "Kártya augmentálása...");
        cv::Mat augmented_image = augmentCard(card_images_[i], background);
        RCLCPP_INFO(this->get_logger(), "Generált kép mentése...");
        saveGeneratedImage(augmented_image, i);
    }
    RCLCPP_INFO(this->get_logger(), "processAllCards befejeződött.");
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 3) {
        std::cerr << "Használat: " << argv[0] << " <kártyák mappa elérési útja> <output mappa elérési útja>\n";
        return 1;
    }

    std::string cards_dir = argv[1];
    std::string output_dir = argv[2];

    // Ellenőrizzük, hogy a kártyák mappája létezik-e
    if (!fs::exists(cards_dir) || !fs::is_directory(cards_dir)) {
        std::cerr << "Hiba: A megadott kártyák mappa nem létezik vagy nem mappa: " << cards_dir << "\n";
        return 1;
    }
    // Ellenőrizzük, hogy az output mappa létezik-e, ha nem, akkor létrehozzuk
    if (!fs::exists(output_dir)) {
        std::cout << "Az output mappa nem létezik, létrehozás: " << output_dir << "\n";
        fs::create_directories(output_dir);  
    } else if (!fs::is_directory(output_dir)) {
        std::cerr << "Hiba: A megadott output útvonal nem mappa: " << output_dir << "\n";
        return 1;
    }
    
    auto node = std::make_shared<CardAugmenterNode>(cards_dir, output_dir);
    node->processAllCards();
    
    rclcpp::shutdown();
    return 0;
}
