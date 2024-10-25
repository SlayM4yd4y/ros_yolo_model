#include "card_gen.hpp"


/* !!!!! to be implemented
//initialize the static variable 'counter' which is responsible for the card number
int CardGenerator::counter = 1;
*/
CardGenerator::CardGenerator() : Node("card_generator")
{
    RCLCPP_INFO(this->get_logger(), "Card Generator Node Started");
}

void CardGenerator::generate_card(const std::string& output_dir, int count)
{
    RCLCPP_INFO(this->get_logger(), "Generating card...");

    // Check if the output directory exists, create it if not
    if (!std::filesystem::exists(output_dir)) {
        RCLCPP_INFO(this->get_logger(), "Directory doesn't exist...");
        std::filesystem::create_directories(output_dir);
    }

    // generating the dummy card
    cv::Mat card(359, 561, CV_8UC3, cv::Scalar(255, 255, 255));

    auto ft2 = cv::freetype::createFreeType2();
    ft2->loadFontData("fonts/Montserrat/Montserrat-VariableFont_wght.ttf", 0);

    ft2->putText(card, "HALLGATÓI", cv::Point(49, 132), 30, cv::Scalar(205, 182, 0), 1, cv::LINE_AA, false);
    ft2->putText(card, "KÁRTYA", cv::Point(50, 168), 28, cv::Scalar(205, 182, 0), -1, cv::LINE_AA, false);
    
    // variable with the card filename
    std::string card_filename = output_dir + "/card" + std::to_string(count) + ".png";

    // saving the card to the output directory
    cv::imwrite(card_filename, card);
    RCLCPP_INFO(this->get_logger(), "Card saved to %s", output_dir.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto card_gen = std::make_shared<CardGenerator>();


    std::string package_name = "ros_yolo_model";
    std::string package_path = ament_index_cpp::get_package_share_directory(package_name);
    std::cout << "The package path is: " << package_path << std::endl;

    //to be made with static variable
    for (int i = 0; i < 5; i++){
        card_gen->generate_card(package_path + "/../../../../src/ros_yolo_model/generated_cards", i);
    }

    rclcpp::spin(card_gen);
    rclcpp::shutdown();
    return 0;
}