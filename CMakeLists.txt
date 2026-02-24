
option(MC_MAKE_LIBRARIES "Build as libraries instead of executables" OFF)
set(MC_MAKE_LIBRARY_NAME "mc_firmware" CACHE STRING "Name of the library to create when MC_MAKE_LIBRARIES is ON")

if ( MC_MAKE_LIBRARIES)
  set(UPPER_PROJECT_NAME ${MC_MAKE_LIBRARY_NAME})
else()
  set(UPPER_PROJECT_NAME ${CMAKE_PROJECT_NAME})
endif()

target_include_directories(${UPPER_PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mc_drivers_common/include>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mc_drivers_common/src>  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/${UPPER_PROJECT_NAME}> 
)


target_sources(${UPPER_PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/can_mc_driver.cpp
)

