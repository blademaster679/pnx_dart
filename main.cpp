#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "light_detection_base.hpp"
#include "pnp_solver.hpp"

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

    std::array<double, 9> camera_matrix = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    std::vector<double> dist_coeffs = {0.0, 0.0, 0.0, 0.0, 0.0};
    PnPSolver solver(camera_matrix, dist_coeffs);
    cv::Mat rvec, tvec;
    double distance, angle;
    for (const auto& light : lights){
        if (solver.solvePnP(light, rvec, tvec)){
            distance = solver.getDistance(light, rvec, tvec);
            angle = solver.getAngle(light, rvec, tvec);
        }
    }


    return 0;
}