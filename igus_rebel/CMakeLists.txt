cmake_minimum_required(VERSION 3.8)
project(igus_rebel)

# Set the required C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(igus_rebel_msgs REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Include directories
include_directories(
  include
)

# Add hardware interface shared library
add_library(${PROJECT_NAME} SHARED
  src/Rebel.cpp
  src/RebelSocket.cpp
  src/CriMessages.cpp
)

target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC)

# Add library dependencies
ament_target_dependencies(${PROJECT_NAME}
  pluginlib hardware_interface igus_rebel_msgs
)

pluginlib_export_plugin_description_file(hardware_interface rebel.xml)

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the plugin description xml file
install(FILES
  rebel.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install package folders
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
