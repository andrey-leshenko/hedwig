cmake_minimum_required(VERSION 2.8.12)

project(stabilization)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

add_executable(stabilize ${SOURCES})
target_link_libraries(stabilize ${OpenCV_LIBS})
target_compile_features(stabilize PRIVATE cxx_uniform_initialization)
