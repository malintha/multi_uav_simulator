cmake_minimum_required(VERSION 3.10)
project(geo_controller)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        simulator_utils
        )

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES geo_controller
        CATKIN_DEPENDS message_runtime roscpp
)

include_directories(
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

#find_package(simulator_utils)

#add_executable(geo_controller
#        include/geo_controller/controllerImpl.h
#        src/controllerImpl.cpp include/geo_controller/definitions.h)

add_library(${PROJECT_NAME}
        src/controllerImpl.cpp
        src/definitions.cpp
        )