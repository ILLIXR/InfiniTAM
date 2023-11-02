##########################################
# Top-level CMakeLists.txt for InfiniTAM #
##########################################

cmake_minimum_required(VERSION 3.16)

PROJECT(InfiniTAM LANGUAGES CUDA CXX)
set(CMAKE_CUDA_COMPILER "/usr/local/cuda/bin/nvcc")
#set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)


set(PLUGIN_NAME plugin.infinitam${ILLIXR_BUILD_SUFFIX})

#find_package( OpenCV 3.4.6 EXACT REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} ) 

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR} ${ILLIXR_ROOT} "/usr/local/cuda/include" "/usr/include/opencv4")
message(ILLIXR_ROOT="${ILLIXR_ROOT}")

# Prefer GLVND
if (POLICY CMP0072)
  cmake_policy (SET CMP0072 NEW)
endif(POLICY CMP0072)

set(CMAKE_C_COMPILER "/usr/bin/clang-14")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-14")

#################################
# Specify the CMake module path #
#################################

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#################################
# Add additional compiler flags #
#################################

IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -std=c++17 -Wno-deprecated-declarations -Wno-unused-function -I /usr/local/cuda/include")
    #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++ -std=c++17 -Wno-deprecated-declarations -Wno-unused-function ")
ENDIF()

IF(NOT MSVC_IDE)
  SET(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  SET(CMAKE_CXX_FLAGS "-fPIC -O3 -std=c++17 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
  #SET(CMAKE_CXX_FLAGS "-fPIC -g ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF()

#########################
# Suppress auto-linking #
#########################

IF(MSVC_IDE)
  ADD_DEFINITIONS(-DUSING_CMAKE=1)
ENDIF()

######################
# Add subdirectories #
######################

ADD_SUBDIRECTORY(Apps)
ADD_SUBDIRECTORY(FernRelocLib)
ADD_SUBDIRECTORY(InputSource)
ADD_SUBDIRECTORY(ITMLib)
ADD_SUBDIRECTORY(MiniSlamGraphLib)
ADD_SUBDIRECTORY(ORUtils)

#add_library(plugin STATIC plugin.cpp)
message(STATUS "CUDA Include Directory: ${CUDA_INCLUDE_DIRS}")
message(STATUS "CUDA Library Directory: ${CUDA_LIBRARY_DIRS}")

add_library(${PLUGIN_NAME} SHARED plugin.cpp)
TARGET_LINK_LIBRARIES(plugin CLIEngine InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib) 
#TARGET_LINK_LIBRARIES(plugin UIEngine InputSource ITMLib MiniSlamGraphLib ORUtils FernRelocLib) 
target_link_libraries(plugin Eigen3::Eigen)  
#target_link_libraries(plugin ${OpenCV_LIBS} Eigen3::Eigen)  
install(TARGETS ${PLUGIN_NAME} DESTINATION lib)
