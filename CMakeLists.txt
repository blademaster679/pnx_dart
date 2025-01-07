cmake_minimum_required(VERSION 3.10)

# set the project name 工程名称
project(pnx_dart)

# set the C++ standard 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

file(GLOB_RECURSE DETECTOR_FILES 
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    )

# include directories 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR}/cameraOpen/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# find the OpenCV yaml-cpp Eigen Ceres library 查找OpenCV库和yaml-cpp库
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED )
find_package(Ceres REQUIRED)

# add the executable 添加可执行文件
add_executable(main main.cpp ${DETECTOR_FILES})

# link the OpenCV yaml-cpp Eigen Ceres library 链接OpenCV库和yaml-cpp库
# link the required libraries 链接必要的库
target_link_libraries(main
    
    ${OpenCV_LIBS}
    yaml-cpp
    Eigen3::Eigen
    ${CERES_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/cameraOpen/64/libMvCameraControl.so
    ${PROJECT_SOURCE_DIR}/lib/libserial_driver.so
    ${PROJECT_SOURCE_DIR}/lib/libio_context.so
    ${PROJECT_SOURCE_DIR}/lib/libserial_driver_nodes.so
)