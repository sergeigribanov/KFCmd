cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(kfcmd VERSION 1.0)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(kfbase REQUIRED)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/env.sh.in env.sh)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rootlogon.C.in rootlogon.C)

add_library(kfcmd SHARED ${SOURCES})

target_include_directories(kfcmd
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kfcmd
  PRIVATE
  kfbase::kfbase_core)

set(PUBLIC_HEADERS ${HEADERS})

set_target_properties(kfcmd
  PROPERTIES
  PUBLIC_HEADER "${PUBLIC_HEADERS}")

# install(TARGETS kfcmd
#   EXPORT kfcmdTargets
#   LIBRARY DESTINATION lib
#   PUBLIC_HEADER DESTINATION include/kfcmd)
# install(EXPORT kfcmdTargets
#   FILE kfcmdTargets.cmake
#   DESTINATION lib/cmake)
# include(CMakePackageConfigHelpers)
# configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/kfcmdConfig.cmake.in
#   "${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfig.cmake"
#   INSTALL_DESTINATION "lib/cmake/example"
#   NO_SET_AND_CHECK_MACRO
#   NO_CHECK_REQUIRED_COMPONENTS_MACRO)
# write_basic_package_version_file(
#   "${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfigVersion.cmake"
#   VERSION "${kfcmd_VERSION_MAJOR}.${kfcmd_VERSION_MINOR}"
#   COMPATIBILITY AnyNewerVersion)
# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfig.cmake
#   DESTINATION lib/cmake)
# install(FILES ${PROJECT_BINARY_DIR}/env.sh DESTINATION env)
# install(FILES ${PROJECT_BINARY_DIR}/rootlogon.C DESTINATION env)
# export(EXPORT kfcmdTargets
#   FILE "${CMAKE_CURRENT_BINARY_DIR}/kfcmdTargets.cmake")
# export(PACKAGE kfcmd)
