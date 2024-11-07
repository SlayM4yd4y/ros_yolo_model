#include "card_gen.hpp"


/* !!!!! to be implemented
//initialize the static variable 'counter' which is responsible for the card number
int CardGenerator::counter = 1;
*/
CardGenerator::CardGenerator() : Node("card_generator")
{
    RCLCPP_INFO(this->get_logger(), "Card Generator Node Started");
    //initialize the vectors with the names, ncodes, id, images, barcodes and cardId
    names = {"John Doe", "Jane Doe", "John Smith"};
    nCodes = {"A1B2C3", "D4E5F6", "G7H8I9"};
    ids = {"56981237", "66982238", "45981239"};
    cardIds = {"15268647", "25268648", "85268649"};
    images = {package_path()+"/img/card_gen/portrait1.png", package_path()+"/img/card_gen/portrait2.png", package_path()+"/img/card_gen/portrait3.png"};
    barcodes = {package_path()+"/img/card_gen/rsz_barcode1.png", package_path()+"/img/card_gen/rsz_barcode2.png", package_path()+"/img/card_gen/rsz_barcode3.png"};
    cardTemplates = {package_path()+"/img/card_gen/student_c.png", package_path()+"/img/card_gen/employee_c.png"};
}

void CardGenerator::generate_card(const std::string& output_dir)
{
    RCLCPP_INFO(this->get_logger(), "Generating card...");

    // Check if the output directory exists, create it if not
    if (!std::filesystem::exists(output_dir)) {
        RCLCPP_INFO(this->get_logger(), "Directory doesn't exist...");
        std::filesystem::create_directories(output_dir);
    }

    // generating the dummy card
    int counter = 0;




    for (int i = 0; i < static_cast<int>(cardTemplates.size()); i++){
        for (int ii = 0; ii < static_cast<int>(names.size()); ii++){
            for (int iii = 0; iii < static_cast<int>(cardIds.size()); iii++){
                for (int iv = 0; iv < static_cast<int>(images.size()); iv++){
                    for (int v = 0; v < static_cast<int>(barcodes.size()); v++){
                        for (int vi = 0; vi < static_cast<int>(ids.size()); vi++){

                            

                            cv::Mat card(359, 553, CV_8UC3, cv::Scalar(255, 255, 255));
                            auto ft2 = cv::freetype::createFreeType2();
                            ft2->loadFontData(package_path() + "/fonts/Montserrat/Montserrat-VariableFont_wght.ttf", 0);

                            cv::Mat card_template = cv::imread(cardTemplates[i]);
                            if (cardTemplates.empty()){
                                RCLCPP_ERROR(this->get_logger(), "Failed to load template image");
                                return;
                            }
                            cv::Mat image = cv::imread(images[iv]);
                            if (images.empty()){
                                RCLCPP_ERROR(this->get_logger(), "Failed to load an image from the corresponding vector");
                                return;
                            }
                            cv::Mat barcode = cv::imread(barcodes[v]);
                            if (barcodes.empty()){
                                RCLCPP_ERROR(this->get_logger(), "Failed to load a barcode from the corresponding vector");
                                return;
                            }
                            if (cardTemplates[i].find("student") != std::string::npos){

                                cv::Rect ct(cv::Point(0, 0), card_template.size());
                                cv::Rect bc(cv::Point(41, 29), barcode.size());
                                cv::Rect pt(cv::Point(382, 122), image.size());
                                card_template.copyTo(card(ct));
                                barcode.copyTo(card(bc));
                                image.copyTo(card(pt));

                                ft2->putText(card, names[ii], cv::Point(61, 297), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
                                ft2->putText(card, "H" + cardIds[iii], cv::Point(158, 230), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
                                ft2->putText(card, nCodes[vi], cv::Point(157, 260), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);

                                
                            }else if (cardTemplates[i].find("employee") != std::string::npos){
                                cv::Rect ct(cv::Point(0, 0), card_template.size());
                                cv::Rect bc(cv::Point(49, 22), barcode.size());
                                cv::Rect pt(cv::Point(386, 123), image.size());
                                card_template.copyTo(card(ct));
                                barcode.copyTo(card(bc));
                                image.copyTo(card(pt));

                                ft2->putText(card, names[ii], cv::Point(65, 297), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
                                ft2->putText(card, "A" + cardIds[iii], cv::Point(165, 230), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
                                ft2->putText(card, ids[vi], cv::Point(165, 260), 15, cv::Scalar(0, 0, 0), 1, cv::LINE_AA, false);
                                
                                
                            }
                            // variable with the card filename
                            std::string card_filename = output_dir + "/card" + std::to_string(counter) + ".png";

                            // saving the card to the output directory
                            cv::imwrite(card_filename, card);
                            RCLCPP_INFO(this->get_logger(), "Card saved to %s", output_dir.c_str());
                            ++counter;
                        }
                    }
                }
            }
        }
    }

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
    card_gen->generate_card(package_path() + "/generated_cards");

    rclcpp::spin_some(card_gen);
    rclcpp::shutdown();
    return 0;
}