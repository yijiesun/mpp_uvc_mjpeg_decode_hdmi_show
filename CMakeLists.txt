cmake_minimum_required(VERSION 2.8)

project(mpp_uvc_mjpeg_decode_hdmi_show)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-fpermissive) 
set(SOURCE_FILES
        main.cpp 
        v4l2/v4l2.cpp
        thread/Thread.cpp
        rkdrm/bo.c
        rkdrm/dev.c
        mpp/utils/utils.c
        rkdrm/modeset.c)

include_directories(
        /usr/local/include/
        /usr/local/include/groupsock
        /usr/local/include/liveMedia
        /usr/local/include/BasicUsageEnvironment
        /usr/local/include/UsageEnvironment
        /usr/include/rockchip
        /usr/local/include/libdrm
        mpp/osal/inc
        mpp/utils
        )     
add_executable(mpp_uvc_mjpeg_decode_hdmi_show ${SOURCE_FILES})
target_link_libraries(mpp_uvc_mjpeg_decode_hdmi_show pthread drm rockchip_mpp avformat avcodec -lBasicUsageEnvironment -lUsageEnvironment -lliveMedia -lgroupsock)