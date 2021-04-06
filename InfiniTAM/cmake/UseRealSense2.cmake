#######################
# UseRealSense2.cmake #
#######################

OPTION(WITH_REALSENSE2 "Build with Intel RealSense SDK 2 support?" ON)

IF(WITH_REALSENSE2)
  FIND_PACKAGE(realsense2 REQUIRED)
  INCLUDE_DIRECTORIES(${realsense2_INCLUDE_DIR})
  ADD_DEFINITIONS(-DCOMPILE_WITH_RealSense2)
ENDIF()
