#include "pnp_solver.hpp"
#include "light_detection_base.hpp"

PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix, const std::vector<double> &dist_coeffs)
: camera_matrix(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  distortion_coefficients(cv::Mat(1,5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone()){
    constexpr double radius = CIRCLE_RADIUS / 1000;// convert to meters
    
    
    circle_points.emplace_back(cv::Point3f(0, 0, radius));
    circle_points.emplace_back(cv::Point3f(0, radius, 0));
    circle_points.emplace_back(cv::Point3f());
    circle_points.emplace_back(cv::Point3f());}