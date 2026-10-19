cmake_minimum_required(VERSION 4.0)

project(my_visualization LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY launch rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
