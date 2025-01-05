#include "light_detection_base.hpp"

cv::Mat Detector::binary_image(const cv::Mat &color_image){
    if (color_image.type() != CV_8UC3 && color_image.type() != CV_8UC4){
        throw std::invalid_argument("Input image must be 3 or 4 channel image, either CV_8UC3 or CV_8UC4.");
    }
    cv::Mat grey_image;//convert color image to grey image
    cv::cvtColor(color_image, grey_image, cv::COLOR_BGR2GRAY);
    cv::imshow("grey_image", grey_image);//调试
    cv::waitKey(0);
    cv::Mat binary_image;//convert grey image to binary image
    cv::threshold(grey_image, binary_image, binary_threshold, 255, cv::THRESH_BINARY);
    cv::imshow("binary_image", binary_image);//调试
    cv::waitKey(0);
    cv::Mat gradient_image;//close operation to binary image 形态学操作
    int kernal_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernal_size, kernal_size));
    cv::morphologyEx(binary_image, gradient_image, cv::MORPH_CLOSE, element); 
    cv::imshow("gradient_image", gradient_image);//调试
    cv::waitKey(0);
    return gradient_image;
}

void Detector::Light::initializeLight(const cv::Point2f &center, float radius){
    top = cv::Point2f(center.x, center.y - radius);//y坐标向下为正
    bottom = cv::Point2f(center.x, center.y + radius);
    left = cv::Point2f(center.x - radius, center.y);
    right = cv::Point2f(center.x + radius, center.y);
    width = 2 * radius;
    height = 2 * radius;
    title_angle = 0;
    std::cout << "top: " << top << std::endl;//调试
    std::cout << "bottom: " << bottom << std::endl;//调试
    std::cout << "left: " << left << std::endl;//调试
    std::cout << "right: " << right << std::endl;//调试
}

bool isLight(const Detector::Light &lights){
    float minus = abs(lights.bottom.y - lights.top.y);
    std::cout << "minus: " << minus << std::endl;//调试
    bool minus_ok = minus > lights.min_minus && minus < lights.max_minus;
    return minus_ok;
}



std::vector<Detector::Light> Detector::find_lights(const cv::Mat &color_image, const cv::Mat &binary_image){
    CV_Assert(binary_image.type() == CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;
    cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "contours size: " << contours.size() << std::endl;//调试
    std::vector<Detector::Light> lights;
    for (const auto &contour : contours){
        if (contour.size() < 10){
            continue;
        }
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        if (radius < 20){
            continue;
        }
        Detector::Light light(center, radius);
        if (isLight(light)){
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.x >= 0 && rect.width >= 0 && rect.x + rect.width <= color_image.cols && rect.y >= 0 && rect.height >= 0 && rect.y + rect.height <= color_image.rows){
                int sum_r = 0, sum_g = 0, sum_b = 0;
                auto roi = color_image(rect);
                for (int i = 0; i < roi.rows; i++){
                    for (int j = 0; j < roi.cols; j++){
                        if (cv::pointPolygonTest(contour, cv::Point2f(rect.x + j, rect.y + i), false) >= 0){
                            sum_b += roi.at<cv::Vec3b>(i, j)[BLUE];
                            sum_g += roi.at<cv::Vec3b>(i, j)[GREEN];
                            sum_r += roi.at<cv::Vec3b>(i, j)[RED];
                        }
                    }
                }
                if (sum_g > sum_r && sum_g > sum_b){
                    lights.emplace_back(light);
                }
            }
        }
    }
    return lights;
}