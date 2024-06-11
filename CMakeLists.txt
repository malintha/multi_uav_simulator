cmake_minimum_required(VERSION 3.8)
project(mavswarm2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  endif()
  
  
  # find dependencies
  find_package(Armadillo REQUIRED)
  find_package(GSL REQUIRED)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(simulator_interfaces REQUIRED)
  # # find_package(simulator_utils REQUIRED)
  # # find_package(controller REQUIRED)
  find_package(std_msgs REQUIRED)
  # find_package(GSL REQUIRED)

  find_package(geometry_msgs REQUIRED)
  # find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  
  
# add_library(${PROJECT_NAME} SHARED
#   src/quadrotor.cpp
# )

add_executable(mavswarm2 src/quadrotor.cpp)

ament_target_dependencies(${PROJECT_NAME}
    geometry_msgs
    rclcpp
    tf2
    tf2_ros
    visualization_msgs
    simulator_interfaces
    
    # turtlesim
)

# ament_target_dependencies(mavswarm2  
# rclcpp
# geometry_msgs
# visualization_msgs
# std_msgs
# tf2 
# tf2_ros
# simulator_interfaces 
# )

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)
#   # EXPORT ${PROJECT_NAME}
#   # LIBRARY DESTINATION lib
#   # ARCHIVE DESTINATION lib
#   # RUNTIME DESTINATION bin
#   # INCLUDES DESTINATION include
# )
ament_package()

target_link_libraries(mavswarm2
        GSL::gsl
        GSL::gslcblas)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

include_directories(
        include/${PROJECT_NAME}
        # include/gcopter
        # include/misc
)
