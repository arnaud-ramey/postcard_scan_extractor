PROJECT(postcard_scan_extractor)
cmake_minimum_required(VERSION 2.6)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_VERBOSE_MAKEFILE ON)

FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/postcard_scan_extractor_path.h.in"
               "${PROJECT_BINARY_DIR}/postcard_scan_extractor_path.h")

ADD_EXECUTABLE(postcard_scan_extractor postcard_scan_extractor.cpp
                                       postcard_scan_extractor.h)
TARGET_LINK_LIBRARIES( postcard_scan_extractor ${OpenCV_LIBS})

