cmake_minimum_required(VERSION 3.16)
project(camera_calibration_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d highgui videoio)

add_library(calibration_common STATIC
    src/config.cpp
)
target_include_directories(calibration_common PUBLIC include)
target_link_libraries(calibration_common PUBLIC ${OpenCV_LIBS})

add_executable(rm_camera_calibrate src/calibrate.cpp)
target_link_libraries(rm_camera_calibrate PRIVATE calibration_common ${OpenCV_LIBS})

add_executable(rm_undistort src/undistort.cpp)
target_link_libraries(rm_undistort PRIVATE calibration_common ${OpenCV_LIBS})
