cmake_minimum_required(VERSION 3.16)

# Merge local WiFi credentials (gitignored) after defaults when present.
set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.local")
    list(APPEND SDKCONFIG_DEFAULTS "sdkconfig.local")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(reachy_motion_controller)
