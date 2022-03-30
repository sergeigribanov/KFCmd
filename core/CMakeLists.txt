file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/kfcmd/core/*.hpp)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(kfcmd_core SHARED ${SOURCES})
add_library(kfcmd::core ALIAS kfcmd_core)
target_include_directories(kfcmd_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(kfcmd_core PUBLIC kfbase::kfbase_core)
set_target_properties(kfcmd_core
  PROPERTIES
  PUBLIC_HEADER "${HEADERS}")

install(TARGETS kfcmd_core
  EXPORT kfcmdTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kfcmd/core)
