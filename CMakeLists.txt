cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(KFCmd VERSION 1.0)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(CCGO REQUIRED)
find_package(KFBase REQUIRED)
list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})
find_package(ROOT REQUIRED COMPONENTS RIO Net Core Eve EG Geom Ged RGL)
include(${ROOT_USE_FILE})
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)
set(LIB_SOURCES ${SOURCES})
list(REMOVE_ITEM LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/test.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/env.sh.in env.sh)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rootlogon.C.in rootlogon.C)
add_library(kfcmd SHARED ${LIB_SOURCES})
set_property(TARGET kfcmd PROPERTY CXX_STANDARD 11)
target_include_directories(kfcmd
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  ${CCGO_INCLUDE_DIRS}
  ${KFBase_INCLUDE_DIRS})
target_link_libraries(kfcmd
  PRIVATE
  Eigen3::Eigen
  ${ROOT_LIBRARIES}
  ${CCGO_LIBRARIES}
  ${KFBase_LIBRARIES})
set(PUBLIC_HEADERS ${HEADERS})
set_target_properties(kfcmd
  PROPERTIES
  PUBLIC_HEADER "${PUBLIC_HEADERS}")
install(TARGETS kfcmd
  EXPORT KFCmdTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/KFCmd)
install(EXPORT KFCmdTargets
  FILE KFCmdTargets.cmake
  DESTINATION lib/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/KFCmdConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/KFCmdConfig.cmake"
  INSTALL_DESTINATION "lib/cmake/example"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/KFCmdConfigVersion.cmake"
  VERSION "${KFCmd_VERSION_MAJOR}.${KFCmd_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/KFCmdConfig.cmake
  DESTINATION lib/cmake)
install(FILES ${PROJECT_BINARY_DIR}/env.sh DESTINATION env)
install(FILES ${PROJECT_BINARY_DIR}/rootlogon.C DESTINATION env)
export(EXPORT KFCmdTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/KFCmdTargets.cmake")
export(PACKAGE KFCmd)
