#ifndef MESSAGE_HPP
#define MESSAGE_HPP
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

namespace rm_dart{
class message{
public:
    message() = default;
    double distance;
    double angle;
};    
}
#endif