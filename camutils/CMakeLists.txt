cmake_minimum_required(VERSION 3.2)

project(camutils)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# TODO(Andrey): Rewrite when I am better at CMake.
add_executable(camera_calibrate camera_calibrate.cpp)
target_link_libraries(camera_calibrate ${OpenCV_LIBS})

add_executable(camera_framerate camera_framerate.cpp)
target_link_libraries(camera_framerate ${OpenCV_LIBS})

add_executable(camera_latency camera_latency.cpp)
target_link_libraries(camera_latency ${OpenCV_LIBS})
