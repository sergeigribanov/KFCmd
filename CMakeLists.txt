cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(KFCmd)
option(EIGEN3_INCLUDE_DIR "Path to the Eigen3 include directory")
option(CCGO_DIR "Path to the CCGO installation")
option(KFBASE_DIR "Path to the KFBase installation")
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
if (NOT EIGEN3_INCLUDE_DIR)
  if (DEFINED ENV{EIGEN3_INCLUDE_DIR})
    set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
  else()
    message(FATAL_ERROR "EIGEN3_INCLUDE_DIR option is not set")
  endif()
endif()
if (NOT CCGO_DIR)
  if (DEFINED ENV{CCGO_DIR})
    set(CCGO_DIR $ENV{CCGO_DIR})
  else()
    message(FATAL_ERROR "CCGO_DIR option is not set")
  endif()
endif()
if (NOT KFBASE_DIR)
  if (DEFINED ENV{KFBASE_DIR})
    set(KFBASE_DIR $ENV{KFBASE_DIR})
  else()
    message(FATAL_ERROR "KFBASE_DIR option is not set")
  endif()
endif()
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
target_include_directories(kfcmd
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CCGO_DIR}/include
  ${KFBASE_DIR}/include)
target_link_libraries(kfcmd
  ${CCGO_DIR}/lib/libccgo.so
  ${ROOT_LIBRARIES}
  ${KFBASE_DIR}/lib/libkfbase.so)
set(PUBLIC_HEADERS ${HEADERS})
set_target_properties(kfcmd
  PROPERTIES
  PUBLIC_HEADER "${PUBLIC_HEADERS}")
install(TARGETS kfcmd
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/KFCmd)
install(FILES ${PROJECT_BINARY_DIR}/env.sh DESTINATION env)
install(FILES ${PROJECT_BINARY_DIR}/rootlogon.C DESTINATION env)
