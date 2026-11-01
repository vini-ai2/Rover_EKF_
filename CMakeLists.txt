cmake_minimum_required(VERSION 3.16)
project(RoverEKF VERSION 1.0.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(RoverEKF INTERFACE)
add_library(RoverEKF::RoverEKF ALIAS RoverEKF)
target_compile_features(RoverEKF INTERFACE cxx_std_11)
target_include_directories(RoverEKF INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(RoverEKF INTERFACE Eigen3::Eigen)

install(TARGETS RoverEKF EXPORT RoverEKFTargets)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT RoverEKFTargets
  FILE RoverEKFTargets.cmake
  NAMESPACE RoverEKF::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoverEKF
)
configure_package_config_file(
  cmake/RoverEKFConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RoverEKFConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoverEKF
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/RoverEKFConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/RoverEKFConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/RoverEKFConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/RoverEKF
)

include(CTest)
if(BUILD_TESTING)
  add_executable(rover_ekf_tests tests/RoverEKFTests.cpp)
  target_link_libraries(rover_ekf_tests PRIVATE RoverEKF::RoverEKF)
  add_test(NAME rover_ekf_tests COMMAND rover_ekf_tests)
endif()
