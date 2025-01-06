#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <array>
#include "light_detection_base.hpp"
class PnPSolver{
public:
    PnPSolver(
        const std::array<double,9> &camera_matrix,
        const std::vector<double> &distortion_coefficients);
    bool solvePnP(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec);
    float calculateDistanceToCenter(const cv::Point2f &center);
    double getDistance(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec);
    double getAngle(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec);
private:
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    static constexpr float CIRCLE_RADIUS = 27.5;
    std::vector<cv::Point3f> circle_points;
};

#endif