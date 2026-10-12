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
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

file(GLOB SRC src/*.cpp)
file(GLOB LIB
    src/sensor-flow/src/manager.cpp
    src/sensor-flow/src/application.cpp
    src/sensor-flow/src/worker/*.cpp
    src/sensor-flow/src/utility/*.cpp
    src/sensor-flow/src/core/**.cpp
    )
add_executable(my_node ${SRC} ${LIB})

target_include_directories(
    my_node
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor-flow/src
)

target_link_libraries(
    my_node
    PRIVATE
    rclcpp::rclcpp
    std_msgs::std_msgs
    sensor_msgs::sensor_msgs
    tf2::tf2
    rclcpp_action::rclcpp_action
    nav2_msgs::nav2_msgs
)


install(
    TARGETS my_node
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
