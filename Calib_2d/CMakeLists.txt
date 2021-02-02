cmake_minimum_required(VERSION 3.0)
project(Calib_2d)  #工程名

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_FLAGS "-std=c++11")  #采用c++ 11标准编译

find_package(OpenCV)

#include_directories(${CMAKE_SOURCE_DIR}/calibration)  #我要引用./calibration/里的头文件，就得把这个头文件目录添加进来

set(src
        ${CMAKE_SOURCE_DIR}/main.cpp
        ${CMAKE_SOURCE_DIR}/calibration.h
        ${CMAKE_SOURCE_DIR}/calibration.cpp)

add_executable(Calib_2d ${src})

target_link_libraries(Calib_2d ${OpenCV_LIBS})  #生成的文件与 CalibrationFuncs库(自己创建的) 相链接
