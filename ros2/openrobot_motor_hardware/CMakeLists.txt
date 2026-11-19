cmake_minimum_required(VERSION 3.16)
project(openrobot_motor_hardware CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The protocol core is the SDK's header-only C++ library. Point
# OPENROBOT_MOTOR_INCLUDE elsewhere if this package is used outside the SDK tree.
set(OPENROBOT_MOTOR_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/../../cpp/include"
    CACHE PATH "openrobot_motor C++ headers")

add_library(openrobot_motor_hardware SHARED src/openrobot_motor_system.cpp)
target_include_directories(openrobot_motor_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${OPENROBOT_MOTOR_INCLUDE}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(openrobot_motor_hardware ${CMAKE_DL_LIBS})
ament_target_dependencies(openrobot_motor_hardware hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface openrobot_motor_hardware.xml)

install(TARGETS openrobot_motor_hardware
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY ${OPENROBOT_MOTOR_INCLUDE}/ DESTINATION include)
install(DIRECTORY description config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(openrobot_motor_hardware)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
