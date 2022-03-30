cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(kfcmd VERSION 1.0)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(kfbase REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/env.sh.in env.sh)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rootlogon.C.in rootlogon.C)

include(GNUInstallDirs)
add_subdirectory(core)
add_subdirectory(hypos)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/kfcmdConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_DATADIR}/kfcmd/cmake"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfigVersion.cmake"
  VERSION "${KFCMD_CORE_VERSION_MAJOR}.${KFCMD_CORE_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion)

export(TARGETS kfcmd_core NAMESPACE kfcmd:: FILE kfcmdTargets.cmake)
export(TARGETS kfcmd_hypos NAMESPACE kfcmd:: APPEND FILE kfcmdTargets.cmake)
export(PACKAGE kfcmd)
install(EXPORT kfcmdTargets NAMESPACE kfcmd:: DESTINATION lib/cmake)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/kfcmdConfig.cmake
  DESTINATION lib/cmake)
install(FILES ${PROJECT_BINARY_DIR}/env.sh DESTINATION bin)
install(FILES ${PROJECT_BINARY_DIR}/rootlogon.C DESTINATION share/kfcmd)
