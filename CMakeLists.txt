cmake_minimum_required(VERSION 3.8)
project(padman_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  eigen3_cmake_module
  Eigen3
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add padman_controller library related compile commands
generate_parameter_library(padman_controller_parameters
  src/padman_controller.yaml
  include/padman_controller/validate_padman_controller_parameters.hpp
)
add_library(
  padman_controller
  SHARED
  src/padman_controller.cpp
)
target_include_directories(padman_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(padman_controller padman_controller_parameters pinocchio::pinocchio)
ament_target_dependencies(padman_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(padman_controller PRIVATE "PADMAN_CONTROLLER_BUILDING_DLL")

ament_target_dependencies(padman_controller Eigen3)

pluginlib_export_plugin_description_file(
  controller_interface padman_controller.xml)

install(
  TARGETS
  padman_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_gmock(test_load_padman_controller test/test_load_padman_controller.cpp)
  target_include_directories(test_load_padman_controller PRIVATE include)
  ament_target_dependencies(
    test_load_padman_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_padman_controller test/test_padman_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/padman_controller_params.yaml)
  target_include_directories(test_padman_controller PRIVATE include)
  target_link_libraries(test_padman_controller padman_controller)
  ament_target_dependencies(
    test_padman_controller
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_padman_controller_preceeding test/test_padman_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/padman_controller_preceeding_params.yaml)
  target_include_directories(test_padman_controller_preceeding PRIVATE include)
  target_link_libraries(test_padman_controller_preceeding padman_controller)
  ament_target_dependencies(
    test_padman_controller_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  padman_controller
)

ament_package()
