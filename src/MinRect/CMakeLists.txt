cmake_minimum_required(VERSION 3.17)
project(MinRect)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(MinRect main.cpp)

target_link_libraries (MinRect ${PCL_LIBRARIES})

install(TARGETS MinRect RUNTIME DESTINATION bin)