CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(PLY2PCD)

SET(CMAKE_BUILD_TYPE "Release")

FIND_PACKAGE(PCL REQUIRED)

#INCLUDE
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

#LINK
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

ADD_EXECUTABLE(${PROJECT_NAME} ply2pcd.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES})