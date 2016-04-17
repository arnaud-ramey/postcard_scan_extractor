PROJECT(postcard_scan_extractor)
cmake_minimum_required(VERSION 2.6)
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings

FIND_PACKAGE( OpenCV REQUIRED )
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/postcard_scan_extractor_path.h.in"
               "${PROJECT_BINARY_DIR}/postcard_scan_extractor_path.h")

ADD_EXECUTABLE(postcard_scan_extractor postcard_scan_extractor.cpp
                                       postcard_scan_extractor.h)
TARGET_LINK_LIBRARIES( postcard_scan_extractor ${OpenCV_LIBS})

