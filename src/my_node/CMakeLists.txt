cmake_minimum_required(VERSION 3.28)

project(my_node)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB SRC src/*.cpp)
# teleopでは自動探索のソースをビルドしない。
list(REMOVE_ITEM SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/governor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/explorer.cpp
)

add_executable(my_node ${SRC} ${LIB})

target_include_directories(
    my_node
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor-flow/install/include
)

target_link_libraries(
    my_node
    PRIVATE
    rclcpp::rclcpp
    std_msgs::std_msgs
    sensor_msgs::sensor_msgs
    tf2::tf2
    tf2_ros::tf2_ros
    nav_msgs::nav_msgs
    geometry_msgs::geometry_msgs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor-flow/install/lib/libsensor-flow.so
)


set_target_properties(my_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")

install(
    FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor-flow/install/lib/libsensor-flow.so
    DESTINATION lib
)

install(
    TARGETS my_node
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config urdf
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
