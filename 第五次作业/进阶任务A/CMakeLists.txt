cmake_minimum_required(VERSION 3.16)
project(rm_vision_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV 4 REQUIRED)
find_package(Threads REQUIRED)

add_library(vision_core
  src/vision_config.cpp
  src/color_detector.cpp
  src/target_tracker.cpp
)
target_include_directories(vision_core PUBLIC include)
target_link_libraries(vision_core PUBLIC ${OpenCV_LIBS} Threads::Threads)

add_executable(rm_vision_sim src/main.cpp)
target_link_libraries(rm_vision_sim PRIVATE vision_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(vision_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(rm_vision_sim PRIVATE -Wall -Wextra -Wpedantic)
elseif(MSVC)
  target_compile_options(vision_core PRIVATE /W4)
  target_compile_options(rm_vision_sim PRIVATE /W4)
endif()
