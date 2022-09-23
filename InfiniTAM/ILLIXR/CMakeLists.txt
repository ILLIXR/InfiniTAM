cmake_minimum_required(VERSION 2.8)
project(plugin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
add_definitions(-DCOMPILEDWITHC17)
find_package(OpenCV 3)

include_directories(
${PROJECT_SOURCE_DIR}
)

add_library(${PROJECT_NAME} SHARED
plugin.cpp
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
-lboost_filesystem
-lboost_serialization
-lcrypto
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})