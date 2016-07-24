cmake_minimum_required(VERSION 2.8.12)

project(camutils)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# TODO(Andrey): Rewrite when I am better at CMake.
add_executable(camera_calibrate camera_calibrate.cpp)
target_link_libraries(camera_calibrate ${OpenCV_LIBS})
target_compile_features(camera_calibrate PRIVATE cxx_uniform_initialization)

add_executable(camera_framerate camera_framerate.cpp)
target_link_libraries(camera_framerate ${OpenCV_LIBS})
target_compile_features(camera_framerate PRIVATE cxx_uniform_initialization)

add_executable(camera_latency camera_latency.cpp)
target_link_libraries(camera_latency ${OpenCV_LIBS})
target_compile_features(camera_latency PRIVATE cxx_uniform_initialization)
