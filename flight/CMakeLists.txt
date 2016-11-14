cmake_minimum_required(VERSION 3.2)

project(flight)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB source_files "*.cpp" "arduino/*.cpp" "instatrack/*.cpp")

add_executable(flight ${source_files})
target_link_libraries(flight ${OpenCV_LIBS})
