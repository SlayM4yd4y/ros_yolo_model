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
    cv::Mat card(359, 553, CV_8UC3, cv::Scalar(255, 255, 255));

    auto ft2 = cv::freetype::createFreeType2();
    ft2->loadFontData(package_path() + "/fonts/Montserrat/Montserrat-VariableFont_wght.ttf", 0);

    cv::Mat card_template = cv::imread(package_path() + "/img/card_gen/card_template.png");
    if (card_template.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load card template image");
        return;
    }

    cv::Mat barcode = cv::imread(package_path() + "/img/card_gen/rsz_barcode1.png");
    if (barcode.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load barcode image");
        return;
    }

    cv::Mat portrait = cv::imread(package_path() + "/img/card_gen/portrait1.png");
    if (portrait.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load portrait image");
        return;
    }
    
    cv::Rect ct(cv::Point(0, 0), card_template.size());
    cv::Rect bc(cv::Point(41, 29), barcode.size());
    cv::Rect pt(cv::Point(382, 122), portrait.size());

    card_template.copyTo(card(ct));
    barcode.copyTo(card(bc));
    portrait.copyTo(card(pt));

    // variable with the card filename
    std::string card_filename = output_dir + "/card" + std::to_string(count) + ".png";

    // saving the card to the output directory
    cv::imwrite(card_filename, card);
    RCLCPP_INFO(this->get_logger(), "Card saved to %s", output_dir.c_str());
}

std::string package_path()
{
    std::string package_name = "ros_yolo_model";
    std::string package_path_shared = ament_index_cpp::get_package_share_directory(package_name);
    std::string package_path = package_path_shared + "/../../../../src/ros_yolo_model";
    std::cout << "The package path is: " << package_path << std::endl;
    return (package_path);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto card_gen = std::make_shared<CardGenerator>();

    //to be made with static variable
    for (int i = 0; i < 5; i++){
        card_gen->generate_card(package_path() + "/generated_cards", i);
    }

    rclcpp::spin_some(card_gen);
    rclcpp::shutdown();
    return 0;
}