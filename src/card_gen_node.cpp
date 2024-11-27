#include "card_gen.hpp"
#include <unordered_map>
#include <get_package_path.hpp>

int CardGenerator::counter = 1;

CardGenerator::CardGenerator() : Node("card_generator") {
    RCLCPP_INFO(this->get_logger(), "Node inicializálása");
    
    std::string pkg_path = package_path();
    names = {"John Doe", "Jane Doe", "John Smith"};
    nCodes = {"A1B2C3", "D4E5F6", "G7H8I9"};
    ids = {"56981237", "66982238", "45981239"};
    cardIds = {"15268647", "25268648", "85268649"};
    images = {pkg_path+"/img/card_gen/portrait1.png", pkg_path+"/img/card_gen/portrait2.png", pkg_path+"/img/card_gen/portrait3.png"};
    barcodes = {pkg_path+"/img/card_gen/rsz_barcode1.png", pkg_path+"/img/card_gen/rsz_barcode2.png", pkg_path+"/img/card_gen/rsz_barcode3.png"};
    cardTemplates = {pkg_path+"/img/card_gen/student_c.png", pkg_path+"/img/card_gen/employee_c.png"};
}

void CardGenerator::generate_card(const std::string& output_dir) {
    RCLCPP_INFO(this->get_logger(), "Kartyak generalasa...");

    // Ellenőrizze, hogy a kimeneti könyvtár létezik-e, ha nem, hozza létre
    if (!std::filesystem::exists(output_dir)) {
        std::filesystem::create_directories(output_dir);
    }

    int total_combinations = cardTemplates.size() * names.size() * nCodes.size() * ids.size() * images.size() * barcodes.size();
    int counter = 0;

    for (int i = 0; i < total_combinations; ++i) {
        // Számolja ki az indexeket a különböző vektorokból
        int template_idx = (i / (names.size() * nCodes.size() * ids.size() * images.size() * barcodes.size())) % cardTemplates.size();
        int name_idx = (i / (nCodes.size() * ids.size() * images.size() * barcodes.size())) % names.size();
        int nCode_idx = (i / (ids.size() * images.size() * barcodes.size())) % nCodes.size();
        int id_idx = (i / (images.size() * barcodes.size())) % ids.size();
        int image_idx = (i / barcodes.size()) % images.size();
        int barcode_idx = i % barcodes.size();

        // Töltse be a kártya sablonját, képet és vonalkódot
        cv::Mat card(359, 553, CV_8UC3, cv::Scalar(255, 255, 255));
        auto ft2 = cv::freetype::createFreeType2();
        ft2->loadFontData(package_path() + "/fonts/Montserrat/Montserrat-VariableFont_wght.ttf", 0);

        cv::Mat card_template = cv::imread(cardTemplates[template_idx]);
        cv::Mat image = cv::imread(images[image_idx]);
        cv::Mat barcode = cv::imread(barcodes[barcode_idx]);

        // Helyezze el a képeket és a szöveget a kártyán
        if (cardTemplates[template_idx].find("student") != std::string::npos) {
            // Hallgatói kártya elrendezése
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
            // Alkalmazotti kártya elrendezése
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

        // Kártáya mentése
        std::string card_filename = output_dir + "/card" + std::to_string(counter++) + ".png";
        cv::imwrite(card_filename, card);
        RCLCPP_INFO(this->get_logger(), "Kartya mentve a kovetkezo helyre:  %s", card_filename.c_str());
    }
}



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto card_gen = std::make_shared<CardGenerator>();

    card_gen->generate_card(package_path() + "/generated_cards");

    rclcpp::spin_some(card_gen);
    rclcpp::shutdown();
    return 0;
}
