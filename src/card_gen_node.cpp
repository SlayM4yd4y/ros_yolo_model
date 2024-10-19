#include "ros_yolo_model/card_gen.hpp"

CardGenerator::CardGenerator() : Node("card_generator")
{
    RCLCPP_INFO(this->get_logger(), "Card Generator Node Started");
}

void CardGenerator::generate_card(const std::string& output_dir)
{
    RCLCPP_INFO(this->get_logger(), "Generating card...");

    // Alap dummy kártya kép generálása OpenCV-vel
    cv::Mat card(400, 640, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::putText(card, "Student Card", cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
    
    // Kártya mentése az output könyvtárba
    cv::imwrite(output_dir + "/card.png", card);
    RCLCPP_INFO(this->get_logger(), "Card saved to %s", output_dir.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto card_gen = std::make_shared<CardGenerator>();

    card_gen->generate_card("output_directory");
    
    rclcpp::spin(card_gen);
    rclcpp::shutdown();
    return 0;
}
