cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_octomap_test)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_compile_options(-std=c++11)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
#link_libraries(${OCTOMAP_LIBRARIES})

add_subdirectory(src)
