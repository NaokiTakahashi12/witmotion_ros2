cmake_minimum_required(VERSION 3.15)
project(witmotion_imu_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    system
    thread
)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(witmotion_standard_protocol_driver_node_params
  src/witmotion_standard_protocol_driver_node_params.yaml
)
ament_auto_add_library(${PROJECT_NAME}_components SHARED
  src/serial_port.cpp
  src/witmotion_serial_imu.cpp
  src/witmotion_standard_protocol_driver_node.cpp
)
target_compile_features(${PROJECT_NAME}_components
  PUBLIC
    cxx_std_20
)
target_link_libraries(${PROJECT_NAME}_components
  Boost::system
  Boost::thread
  Eigen3::Eigen
  witmotion_standard_protocol_driver_node_params
)
set_target_properties(${PROJECT_NAME}_components
  PROPERTIES
    CMAKE_POSITION_INDEPENDENT_CODE ON
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "witmotion_imu_driver::WitmotionStandardProtocolDriverNode"
  EXECUTABLE witmotion_standard_protocol_driver_node
)

install(
  TARGETS
    ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
