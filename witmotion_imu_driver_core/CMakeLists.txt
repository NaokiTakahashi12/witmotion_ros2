cmake_minimum_required(VERSION 3.15)

set(witmotion_imu_driver_core_MAJOR_VERSION 0)
set(witmotion_imu_driver_core_MINOR_VERSION 0)
set(witmotion_imu_driver_core_PATCH_VERSION 0)
set(witmotion_imu_driver_core_VERSION
  ${witmotion_imu_driver_core_MAJOR_VERSION}.${witmotion_imu_driver_core_MINOR_VERSION}.${witmotion_imu_driver_core_PATCH_VERSION})

project(witmotion_imu_driver_core
  VERSION ${witmotion_imu_driver_core_VERSION}
  LANGUAGES
    CXX
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    system
    thread
)
find_package(Eigen3 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR_IN_INTERFACE ON)
set(public_headers
  include/witmotion_imu_driver_core/witmoiton_imu_driver_core.hpp
  include/witmotion_imu_driver_core/standard_protocol.hpp
  include/witmotion_imu_driver_core/serial_port.hpp
  include/witmotion_imu_driver_core/serial_port_options.hpp
  include/witmotion_imu_driver_core/witmotion_serial_imu.hpp
)
add_library(${PROJECT_NAME} SHARED)
target_sources(${PROJECT_NAME}
  PRIVATE
    src/serial_port.cpp
    src/witmotion_serial_imu.cpp
)
target_compile_features(${PROJECT_NAME}
  PUBLIC
    cxx_std_20
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Boost::system
    Boost::thread
    Eigen3::Eigen
)
set_target_properties(${PROJECT_NAME}
  PROPERTIES
    VERSION ${PROJECT_VERSION}
    POSITION_INDEPENDENT_CODE ON
    PUBLIC_HEADER "${public_headers}"
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

export(
  TARGETS
    ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
  EXPORT_LINK_INTERFACE_LIBRARIES
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
install(
  EXPORT ${PROJECT_NAME}Targets
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
