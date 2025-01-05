#include "pnp_solver.hpp"
#include "light_detection_base.hpp"
#include <vector>

PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
: camera_matrix(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  distortion_coefficients(cv::Mat(1,5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()){
    constexpr double radius = CIRCLE_RADIUS / 1000;// convert to meters
    
    //start from the top of the circle and go clockwise
    //Model coordinate system is x-forward, y-left, z-up
    circle_points.emplace_back(cv::Point3f(0, 0, 0));    
    circle_points.emplace_back(cv::Point3f(0, 0, radius));
    circle_points.emplace_back(cv::Point3f(0, radius, 0));
    circle_points.emplace_back(cv::Point3f(0, 0 , -radius));
    circle_points.emplace_back(cv::Point3f(0, -radius, 0));
    }

bool PnPSolver::solvePnP(const Detector::Light & light, cv::Mat & rvec, cv::Mat & tvec){
    std::vector<cv::Point2f> image_points;
    image_points.emplace_back(light.center);
    image_points.emplace_back(light.top);
    image_points.emplace_back(light.right);
    image_points.emplace_back(light.bottom);
    image_points.emplace_back(light.left);
    return cv::solvePnP(
        circle_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        //精度较高，但是速度较慢 1ms
} 

float PnPSolver::calculateDistanceToCenter(const cv::Point2f &center){
    float cx = camera_matrix.at<double>(0, 2);
    float cy = camera_matrix.at<double>(1, 2);
    return cv::norm(center - cv::Point2f(cx, cy));
}

double PnPSolver::getDistance(const Detector::Light &light, cv::Mat &rvec, cv::Mat &tvec){
    std::vector<cv::Point2f> image_points;
    image_points.emplace_back(light.center);
    image_points.emplace_back(light.top);
    image_points.emplace_back(light.right);
    image_points.emplace_back(light.bottom);
    image_points.emplace_back(light.left);
    if (cv::solvePnP(
        circle_points, image_points, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)){
            std::vector<double> tvecVec;
            tvec.copyTo(tvecVec);
            double distance = std::sqrt(tvecVec[0] * tvecVec[0] + 
                                    tvecVec[1] * tvecVec[1] + 
                                    tvecVec[2] * tvecVec[2]);
            std::cout << "distance from the camera to the object: " << distance << std::endl;
            return distance;
        }
    else{
        std::cerr << "solvePnP failed to find a solution." << std::endl;
        return -1;
    }
}