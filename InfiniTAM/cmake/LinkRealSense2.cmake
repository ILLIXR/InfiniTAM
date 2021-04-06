########################
# LinkRealSense2.cmake #
########################

IF(WITH_REALSENSE2)
  TARGET_LINK_LIBRARIES(${targetname} ${realsense2_LIBRARY})
ENDIF()
