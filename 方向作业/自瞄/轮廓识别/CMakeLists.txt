cmake_minimum_required(VERSION 3.16)
project(energy_contour_recognition LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc highgui videoio)

add_executable(energy_contour_recognition
  src/config.cpp
  src/main.cpp
)
target_include_directories(energy_contour_recognition PRIVATE include)
target_link_libraries(energy_contour_recognition PRIVATE ${OpenCV_LIBS})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(energy_contour_recognition PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(energy_contour_recognition PRIVATE /W4)
endif()
