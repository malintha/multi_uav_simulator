cmake_minimum_required(VERSION 3.8)
project(mavswarm2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Armadillo REQUIRED)
find_package(GSL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(simulator_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(glog REQUIRED)

add_executable(mavswarm2 src/quadrotor.cpp)

ament_target_dependencies(mavswarm2
    rclcpp
    geometry_msgs
    tf2
    tf2_ros
    visualization_msgs
    simulator_interfaces
)

target_link_libraries(mavswarm2
    GSL::gsl
    GSL::gslcblas
    glog::glog
)

install(
  TARGETS mavswarm2
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config cf_description
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(
    include/${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_package()
