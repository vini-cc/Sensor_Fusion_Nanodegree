cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


# add_executable (environment src/environment.cpp src/render/render.cpp src/processPointClouds.cpp)
# target_link_libraries (environment ${PCL_LIBRARIES})

# add_executable (main_env src/main_env.cpp src/main_processPcl.cpp src/render/render.cpp src/processPointClouds.cpp)
# target_link_libraries (main_env ${PCL_LIBRARIES})

add_executable (main_env2 src/main_env2.cpp src/main_processPcl.cpp src/render/render.cpp)
target_link_libraries (main_env2 ${PCL_LIBRARIES})

