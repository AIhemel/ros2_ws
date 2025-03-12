cmake_minimum_required(VERSION 3.5)
project(robot_gazebo)

# Default to C11
if(NOT CMAKE_C_STANDARD)
set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY model
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
DESTINATION share/${PROJECT_NAME}
)

ament_package()
