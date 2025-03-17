#include "light_detection_base.hpp"

cv::Mat Detector::binary_image(const cv::Mat &color_image)
{
    if (color_image.type() != CV_8UC3 && color_image.type() != CV_8UC4)
    {
        throw std::invalid_argument("Input image must be 3 or 4 channel image, either CV_8UC3 or CV_8UC4.");
    }
    // 提取绿色通道,Opencv中BGR顺序
    std::vector<cv::Mat> channels;
    cv::split(color_image, channels);
    cv::Mat green_channel = channels[1];
    cv::Mat red_channel = channels[2];
    cv::Mat blue_channel = channels[0];
    cv::Mat greenRegion;
    cv::inRange(green_channel, 100, 255, greenRegion);
    cv::Mat greenCheck;
    cv::bitwise_and(greenRegion, red_channel < 100, greenCheck);
    cv::bitwise_and(greenCheck, blue_channel < 100, greenCheck);

    cv::Mat binary_image; // convert grey image to binary image
    cv::threshold(greenCheck, binary_image, binary_threshold, 255, cv::THRESH_BINARY);
    // cv::imshow("binary_image", binary_image);//调试
    // cv::waitKey(0);
    cv::Mat gradient_image; // close operation to binary image 形态学操作
    int kernal_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernal_size, kernal_size));
    cv::morphologyEx(binary_image, gradient_image, cv::MORPH_GRADIENT, element);
    // cv::imshow("gradient_image", gradient_image);//调试
    // cv::waitKey(0);
    return gradient_image;
}

void Detector::Light::initializeLight(const cv::Point2f &center, float radius)
{
    top = cv::Point2f(center.x, center.y - radius); // y坐标向下为正
    bottom = cv::Point2f(center.x, center.y + radius);
    left = cv::Point2f(center.x - radius, center.y);
    right = cv::Point2f(center.x + radius, center.y);
    width = 2 * radius;
    height = 2 * radius;
    title_angle = 0;
    std::cout << "top: " << top << std::endl;       // 调试
    std::cout << "bottom: " << bottom << std::endl; // 调试
    std::cout << "left: " << left << std::endl;     // 调试
    std::cout << "right: " << right << std::endl;   // 调试
}

bool isLight(const Detector::Light &lights)
{
    float minus = abs(lights.bottom.y - lights.top.y);
    std::cout << "minus: " << minus << std::endl; // 调试
    bool minus_ok = minus > lights.min_minus && minus < lights.max_minus;
    return minus_ok;
}

bool fitCircleLeastSquares(const std::vector<cv::Point> &contourPoints, cv::Point2f &center, float &radius)
{
    int N = contourPoints.size();
    if (N < 3)
    {
        return false;
    }
    // 计算各类求和量（使用 double 提高精度）
    double sumX = 0, sumY = 0;
    double sumX2 = 0, sumY2 = 0, sumXY = 0;
    double sumX2Y2 = 0;   // sum(x_i^2 + y_i^2)
    double sumX_X2Y2 = 0; // sum(x_i * (x_i^2 + y_i^2))
    double sumY_X2Y2 = 0; // sum(y_i * (x_i^2 + y_i^2))

    for (size_t i = 0; i < contourPoints.size(); ++i)
    {
        double x = static_cast<double>(contourPoints[i].x);
        double y = static_cast<double>(contourPoints[i].y);
        double x2 = x * x;
        double y2 = y * y;
        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumXY += x * y;
        double x2y2 = x2 + y2;
        sumX2Y2 += x2y2;
        sumX_X2Y2 += x * x2y2;
        sumY_X2Y2 += y * x2y2;
    }

    // 构建正规方程 (Normal Equation) 的矩阵 M 和向量 Y
    // 方程形式: M * [A, B, C]^T = Y
    // M = | sumX2   sumXY   sumX  |
    //     | sumXY   sumY2   sumY  |
    //     | sumX    sumY    N     |
    // Y = | sumX * (x^2+y^2) |
    //     | sumY * (x^2+y^2) |
    //     | sum(x^2+y^2)     |
    // 其中 A = 2a, B = 2b, C = R^2 - a^2 - b^2 （a,b为圆心坐标, R为半径）
    cv::Mat M = (cv::Mat_<double>(3, 3) << sumX2, sumXY, sumX,
                 sumXY, sumY2, sumY,
                 sumX, sumY, N);
    cv::Mat Y = (cv::Mat_<double>(3, 1) << sumX_X2Y2,
                 sumY_X2Y2,
                 sumX2Y2);
    cv::Mat solution;
    bool solved = solve(M, Y, solution, cv::DECOMP_SVD);
    if (!solved)
    {
        return false; // 解方程失败（可能点全部共线等情况）
    }

    // 提取解并计算圆参数
    double A = solution.at<double>(0, 0);
    double B = solution.at<double>(1, 0);
    double C = solution.at<double>(2, 0);
    double a = A / 2.0;
    double b = B / 2.0;
    double R_squared = a * a + b * b + C;
    if (R_squared < 0)
    {
        return false; // 半径平方为负，说明拟合不出有效圆
    }
    double R = sqrt(R_squared);
    center = cv::Point2f(static_cast<float>(a), static_cast<float>(b));
    radius = static_cast<float>(R);
    return true;
}

std::vector<Detector::Light> Detector::find_lights(const cv::Mat &color_image, const cv::Mat &binary_image)
{
    CV_Assert(binary_image.type() == CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;
    cv::findContours(binary_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "contours size: " << contours.size() << std::endl; // 调试
    std::vector<Detector::Light> lights;
    for (const auto &contour : contours)
    {
        if (contour.size() < 10)
        {
            continue;
        }
        // Draw all contours in the image
        cv::Mat contour_image = color_image.clone();
        cv::drawContours(contour_image, contours, -1, cv::Scalar(0, 255, 0), 2);
        cv::imshow("Contours", contour_image);
        cv::waitKey(1);
        // 调试
        // 遍历轮廓，寻找拟合度最高的圆
        int bestIndex = -1;
        float bestFitScore = -1.0f; // 最佳拟合度得分（0-100%）
        cv::Point2f bestCenter;
        float bestRadius = 0.0f;
        cv::Point2f center;
        float radius;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            // 如果轮廓点太少，跳过
            if (contours[i].size() < 5)
            {
                continue;
            }
            bool ok = fitCircleLeastSquares(contours[i], center, radius);
            if (!ok)
            {
                continue; // 拟合失败，跳过该轮廓
            }
            // 计算当前轮廓的平均相对误差
            double sumError = 0.0;
            for (const cv::Point &p : contours[i])
            {
                double dist = cv::norm(cv::Point2f(p.x, p.y) - center);
                sumError += fabs(dist - radius);
            }
            double meanError = sumError / contours[i].size();
            double errorRatio = meanError / radius;         // 平均误差占半径的比例
            double fitPercent = (1.0 - errorRatio) * 100.0; // 拟合百分比
            if (fitPercent < 0)
                fitPercent = 0; // 最低为0%
            if (fitPercent > 100)
                fitPercent = 100; // 最高为100%
            float fitScore = static_cast<float>(fitPercent);

            // 更新最佳拟合圆信息
            if (fitScore > bestFitScore)
            {
                bestFitScore = fitScore;
                bestIndex = static_cast<int>(i);
                bestCenter = center;
                bestRadius = radius;
            }
        }
        if (bestIndex >= 0) {
            // 在原图上绘制拟合度最高的圆形轮廓及其圆心
            circle(color_image, bestCenter, static_cast<int>(round(bestRadius)), cv::Scalar(0, 0, 255), 2); // 红色圆边框
            circle(color_image, bestCenter, 3, cv::Scalar(255, 0, 0), -1); // 蓝色小圆点标示圆心
    
            // 在图像上绘制文字注释（显示半径和拟合度百分比）
            char label[100];
            snprintf(label, sizeof(label), "R=%.1f, Fit=%.1f%%", bestRadius, bestFitScore);
            cv::Point textPos(static_cast<int>(bestCenter.x - 50), static_cast<int>(bestCenter.y - 10));
            putText(color_image, label, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    
            // 显示结果图像
            cv::imshow("Detected Circle", color_image);
            cv::waitKey(1);
        } else {
            std::cout << "没有找到拟合度达标的圆形对象。" << std::endl;
        }

        Detector::Light light(center, radius);
        if (isLight(light))
        {
            cv::Rect rect = cv::boundingRect(contour);
            if (rect.x >= 0 && rect.width >= 0 && rect.x + rect.width <= color_image.cols && rect.y >= 0 && rect.height >= 0 && rect.y + rect.height <= color_image.rows)
            {
                int sum_r = 0, sum_g = 0, sum_b = 0;
                auto roi = color_image(rect);
                for (int i = 0; i < roi.rows; i++)
                {
                    for (int j = 0; j < roi.cols; j++)
                    {
                        if (cv::pointPolygonTest(contour, cv::Point2f(rect.x + j, rect.y + i), false) >= 0)
                        {
                            sum_b += roi.at<cv::Vec3b>(i, j)[BLUE];
                            sum_g += roi.at<cv::Vec3b>(i, j)[GREEN];
                            sum_r += roi.at<cv::Vec3b>(i, j)[RED];
                        }
                    }
                }
                if (sum_g > sum_r && sum_g > sum_b)
                {
                    lights.emplace_back(light);
                }
            }
        }
    }
    return lights;
}