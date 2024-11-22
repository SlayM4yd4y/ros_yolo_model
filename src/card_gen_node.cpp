#include "card_gen.hpp"
#include <unordered_map>

int CardGenerator::counter = 1;

CardGenerator::CardGenerator() : Node("card_generator") {
    RCLCPP_INFO(this->get_logger(), "Card Generator Node Started");
    
    names = {"John Doe", "Jane Doe", "John Smith"};
    nCodes = {"A1B2C3", "D4E5F6", "G7H8I9"};
    ids = {"56981237", "66982238", "45981239"};
    cardIds = {"15268647", "25268648", "85268649"};
    images = {package_path()+"/img/card_gen/portrait1.png", package_path()+"/img/card_gen/portrait2.png", package_path()+"/img/card_gen/portrait3.png"};
    barcodes = {package_path()+"/img/card_gen/rsz_barcode1.png", package_path()+"/img/card_gen/rsz_barcode2.png", package_path()+"/img/card_gen/rsz_barcode3.png"};
    cardTemplates = {package_path()+"/img/card_gen/student_c.png", package_path()+"/img/card_gen/employee_c.png"};
}

void CardGenerator::generate_card(const std::string& output_dir) {
    RCLCPP_INFO(this->get_logger(), "Generating cards without nested loops...");

    // Ensure the output directory exists
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directories(output_dir);
    }

    int total_combinations = cardTemplates.size() * names.size() * nCodes.size() * ids.size() * images.size() * barcodes.size();
    int counter = 0;

    for (int i = 0; i < total_combinations; ++i) {
        // Calculate indices for each attribute based on `i`
        int template_idx = (i / (names.size() * nCodes.size() * ids.size() * images.size() * barcodes.size())) % cardTemplates.size();
        int name_idx = (i / (nCodes.size() * ids.size() * images.size() * barcodes.size())) % names.size();
        int nCode_idx = (i / (ids.size() * images.size() * barcodes.size())) % nCodes.size();
        int id_idx = (i / (images.size() * barcodes.size())) % ids.size();
        int image_idx = (i / barcodes.size()) % images.size();
        int barcode_idx = i % barcodes.size();

        // Load the template, image, and barcode based on computed indices
        cv::Mat card(359, 553, CV_8UC3, cv::Scalar(255, 255, 255));
        auto ft2 = cv::freetype::createFreeType2();
        ft2->loadFontData(package_path() + "/fonts/Montserrat/Montserrat-VariableFont_wght.ttf", 0);

        cv::Mat card_template = cv::imread(cardTemplates[template_idx]);
        cv::Mat image = cv::imread(images[image_idx]);
        cv::Mat barcode = cv::imread(barcodes[barcode_idx]);

        // Place elements based on template type
        if (cardTemplates[template_idx].find("student") != std::string::npos) {
            // Student card layout
            cv::Rect ct(cv::Point(0, 0), card_template.size());
            cv::Rect bc(cv::Point(41, 29), barcode.size());
            cv::Rect pt(cv::Point(382, 122), image.size());
            card_template.copyTo(card(ct));
            barcode.copyTo(card(bc));
            image.copyTo(card(pt));

            ft2->putText(card, names[name_idx], cv::Point(61, 297), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
            ft2->putText(card, "H" + cardIds[id_idx], cv::Point(158, 230), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
            ft2->putText(card, nCodes[nCode_idx], cv::Point(157, 260), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
        } else {
            // Employee card layout
            cv::Rect ct(cv::Point(0, 0), card_template.size());
            cv::Rect bc(cv::Point(49, 22), barcode.size());
            cv::Rect pt(cv::Point(386, 123), image.size());
            card_template.copyTo(card(ct));
            barcode.copyTo(card(bc));
            image.copyTo(card(pt));

            ft2->putText(card, names[name_idx], cv::Point(65, 297), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
            ft2->putText(card, "A" + cardIds[id_idx], cv::Point(165, 230), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
            ft2->putText(card, ids[id_idx], cv::Point(165, 260), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
        }

        // Save the card
        std::string card_filename = output_dir + "/card" + std::to_string(counter++) + ".png";
        cv::imwrite(card_filename, card);
        RCLCPP_INFO(this->get_logger(), "Card saved to %s", card_filename.c_str());
    }
}


std::string package_path() {
    std::string package_name = "ros_yolo_model";
    std::string package_path_shared = ament_index_cpp::get_package_share_directory(package_name);
    std::string package_path = package_path_shared + "/../../../../src/ros_yolo_model";
    std::cout << "The package path is: " << package_path << std::endl;
    return package_path;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto card_gen = std::make_shared<CardGenerator>();

    card_gen->generate_card(package_path() + "/generated_cards");

    rclcpp::spin_some(card_gen);
    rclcpp::shutdown();
    return 0;
}
