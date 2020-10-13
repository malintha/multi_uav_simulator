cmake_minimum_required(VERSION 2.8.3)

macro(use_cxx11)
    if (CMAKE_VERSION VERSION_LESS "3.1")
        if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
        endif ()
    else ()
        set(CMAKE_CXX_STANDARD 11)
    endif ()
endmacro(use_cxx11)

project(multi_uav_simulator)

find_package(Armadillo REQUIRED)
find_package(GSL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf
        geo_controller
        message_generation
        # quadrotor_msgs
        simulator_utils
        )

catkin_package(
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${geo_controller_INCLUDE_DIRS}
        ${simulator_utils_INCLUDE_DIRS}
)

add_executable(multi_uav_simulator
        src/cfSimUtils.h
        src/DynamicsProvider.cpp
        src/DynamicsProvider.h
        src/cfSimUtils.cpp
        src/Quadrotor.cpp
        src/Quadrotor.h)

target_link_libraries(multi_uav_simulator
        GSL::gsl
        GSL::gslcblas
        ${catkin_LIBRARIES})
