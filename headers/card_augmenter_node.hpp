#ifndef CARD_AUGMENTER_NODE_HPP
#define CARD_AUGMENTER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <random>

class CardAugmenterNode : public rclcpp::Node {
public:
    CardAugmenterNode(const std::string& cards_dir, const std::string& output_dir);
    void processAllCards();
    // Getterek
    inline const std::vector<cv::Mat>& getCardImages() const;
    inline const std::vector<std::string>& getBackgroundImagesPaths() const;
    inline std::string getOutputDir() const;
    // Setterek
    inline void setCardsDir(const std::string& cards_dir);
    inline void setOutputDir(const std::string& output_dir);
private:
    inline cv::Mat loadCardImage(const std::string& image_path);
    cv::Mat augmentCard(const cv::Mat& card, const cv::Mat background);
    inline cv::Mat loadRandomBackground();
    inline void saveGeneratedImage(const cv::Mat& image, int index);
    std::vector<cv::Mat> card_images_;
    std::vector<std::string> background_images_paths_;
    std::string output_dir_;
    mutable std::mt19937 rng_;
};

#endif // CARD_AUGMENTER_NODE_HPP
