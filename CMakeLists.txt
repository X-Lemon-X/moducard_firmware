cmake_minimum_required(VERSION 3.8)


project(moducard_firmware )


add_library(${PROJECT_NAME} STATIC 
  mc_drivers_common/src/mc_common.cpp
)


target_include_directories( ${PROJECT_NAME} PUBLIC
  mc_drivers_common/include
  mc_firmware_common/include
)

