#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "light_detection_base.hpp"

int main(){
    cv::Mat color_image = cv::imread("../images/images2.png",cv::IMREAD_COLOR);
    if (color_image.empty()){
        std::cerr << "read image failed" << std::endl;
        return -1;
    }
    Detector detector;
    cv::Mat binary_image = detector.binary_image(color_image);
    std::vector<Detector::Light> lights = detector.find_lights(color_image, binary_image);
    std::cout << "lights size: " << lights.size() << std::endl;
    cv::imshow("binary_image", binary_image);
    cv::waitKey(0);
    for (const auto& light : lights){
        cv::circle(color_image, light.center, light.width/2 , cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("Detected lights", color_image);
    cv::imwrite("Detected_lights.jpg", color_image);
    cv::waitKey(0);
    return 0;
}