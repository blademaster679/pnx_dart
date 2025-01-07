#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <memory>

// Get config value
#include "yaml-cpp/yaml.h"

//camera open
#include"cameraOpen/include/MvCameraControl.h"
#include"cameraOpen/include/MvErrorDefine.h"
#include"cameraOpen/include/CameraParams.h"
#include"cameraOpen/include/MvISPErrorDefine.h"
#include"cameraOpen/include/MvObsoleteInterfaces.h"
#include"cameraOpen/include/ObsoleteCamParams.h"
#include"cameraOpen/include/PixelType.h"

// dart_detection
#include "light_detection_base.hpp"
#include "pnp_solver.hpp"
#include "message.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

// serialdriver
#include "include/rm_serial_driver.hpp"
#include "include/packet.hpp"
#include "include/crc.hpp"

// // ballistic_calculation
// #include "ballistic_calculation/inlude/ballistic_calculation.hpp"

// 定义一个结构体，用于存储每一帧图像的数据，包含图像和时间戳
struct FrameData
{
    std::shared_ptr<cv::Mat> frame;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};
std::queue<FrameData> frameQueue;     // 用于存储带时间戳的帧
std::mutex frameQueueMutex;           // 互斥锁，保护frameQueue
std::condition_variable frameQueueCV; // 条件变量，用于通知消费者有新的帧，线程同步
const size_t MAX_QUEUE_SIZE = 1;      // 限制队列长度
bool capturing = true;                // 捕捉标志位

std::string error_message;                     // 打开摄像头时的错误信息
rm_serial_driver::RMSerialDriver serialdriver; // 创建一个串口驱动器

// 线程1：
// 打开相机通过回调函数获取图像,下面四个函数为应用到的函数，修改imagecallbackex函数，其他三个直接copy的官方文档
void PressEnterToExit(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;
    fprintf(stderr, "\nPress enter to exit.\n");
    while (getchar() != '\n')
        ;
}
bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        printf("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}
static void __stdcall ImageCallBackEx(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser)
{
    if (pFrameInfo)
    {
        printf("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]\n",
               pFrameInfo->nWidth, pFrameInfo->nHeight, pFrameInfo->nFrameNum);

        cv::Mat mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC1, pData);
        cv::Mat imageRGB;
        cv::cvtColor(mat, imageRGB, cv::COLOR_BayerRG2RGB);

        std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(imageRGB);

        auto timestamp = std::chrono::system_clock::now();

        // 将捕捉到的帧和时间戳放入队列
        std::unique_lock<std::mutex> lock(frameQueueMutex);
        frameQueueCV.wait(lock, []
                  { return frameQueue.size() < MAX_QUEUE_SIZE; });
        frameQueue.push({imgPtr, timestamp});
        lock.unlock();

        // 通知处理线程
        frameQueueCV.notify_one();
    }
}
void videoGet()
{

    int nRet = MV_OK;
    void *handle = NULL;
    do
    {
        // ch:初始化SDK | en:Initialize SDK
        nRet = MV_CC_Initialize();
        if (MV_OK != nRet)
        {
            printf("Initialize SDK fail! nRet [0x%x]\n", nRet);
            break;
        }
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            break;
        }
        printf("Please Intput camera index: ");
        unsigned int nIndex = 0;

        if (nIndex >= stDeviceList.nDeviceNum)
        {
            printf("Intput error!\n");
            break;
        }
        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    printf("Warning: Set Packet Size fail nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                printf("Warning: Get Packet Size fail nRet [0x%x]!\n", nPacketSize);
            }
        }

        // 设置触发模式为off
        // set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
            break;
        }
        // 注册抓图回调
        // register image callback
        nRet = MV_CC_RegisterImageCallBackEx(handle, ImageCallBackEx, handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_RegisterImageCallBackEx fail! nRet [%x]\n", nRet);
            break;
        }
        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }
        PressEnterToExit();
        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            break;
        }
        // 关闭设备
        // close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            break;
        }
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            break;
        }
        handle = NULL;
    } while (0);
    if (handle != NULL)
    {
        MV_CC_DestroyHandle(handle);
        handle = NULL;
    }
    // ch:反初始化SDK | en:Finalize SDK
    MV_CC_Finalize();
    printf("exit\n");
}


// 线程2：
// 处理视频帧图像，进行目标检测，计算距离和角度，发送数据
void processFrames(){
    Detector detector;
    YAML::Node config = YAML::LoadFile("config.yaml");
    std::array<double,9> camera_matrix_data;
    int i = 0;
    for (const auto& row : config["detector"]["camera_matrix"]) {
        for (const auto& element : row) {
            camera_matrix_data[i] = element.as<double>();
            i++;
        }
    }
    
    std::vector<double> dist_coeffs_data;
    for (const auto& element : config["detector"]["dist_coeff"]) {
        dist_coeffs_data.push_back(element.as<double>());
        }
    PnPSolver solver(camera_matrix_data, dist_coeffs_data);
    cv::Mat rvec, tvec;
    double distance, angle;
    while (capturing){
        std::unique_lock<std::mutex> lock(frameQueueMutex);
        frameQueueCV.wait(lock, []{return !frameQueue.empty();});
        FrameData frameData = frameQueue.front();
        frameQueue.pop();
        lock.unlock();
        frameQueueCV.notify_one();

        cv::Mat color_image = *frameData.frame;
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
        for (const auto& light : lights){
            if (solver.solvePnP(light, rvec, tvec)){
                distance = solver.getDistance(light, rvec, tvec);
                angle = solver.getAngle(light, rvec, tvec);
            }
        }
        
        // 发送数据
        rm_dart::message msg;
        msg.angle = angle;
        msg.distance = distance;
        serialdriver.sendData(msg);

        if (cv::waitKey(1) == 'q'){
            capturing = false;
            break;
        }
    }
}

int main(){
    std::thread captureThread(videoGet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::thread processThread(processFrames);
    captureThread.join();
    processThread.join();
    return 0;
}