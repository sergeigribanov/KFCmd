file(GLOB HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/kfcmd/hypos/*.hpp)
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(kfcmd_hypos SHARED ${SOURCES})
add_library(kfcmd::hypos ALIAS kfcmd_hypos)
target_include_directories(kfcmd_hypos
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(kfcmd_hypos PUBLIC kfcmd_core)
set_target_properties(kfcmd_hypos
  PROPERTIES
  PUBLIC_HEADER "${HEADERS}")

install(TARGETS kfcmd_hypos
  EXPORT kfcmdTargets
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kfcmd/hypos)
