cmake_minimum_required(VERSION 3.8)
project(igus_rebel_c_api)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenSSL QUIET)

include(FetchContent)
FetchContent_Declare(
  httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG v0.23.0
)
# Disable ZSTD and Brotli support
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(httplib)

add_executable(arm_api_node src/arm_api_node.cpp)

if(OpenSSL_FOUND)
  target_compile_definitions(arm_api_node PUBLIC CPPHTTPLIB_OPENSSL_SUPPORT)
  target_link_libraries(arm_api_node
    httplib::httplib
    OpenSSL::SSL
    OpenSSL::Crypto
  )
else()
  target_link_libraries(arm_api_node
    httplib::httplib
  )
endif()

target_include_directories(arm_api_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(arm_api_node PUBLIC cxx_std_17)

ament_target_dependencies(
  arm_api_node
  rclcpp
  moveit_ros_planning_interface
  Eigen3
)

install(TARGETS arm_api_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
