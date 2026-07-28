cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(msg_files "msg/AddressBook.msg")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})
# パッケージ使用者のために依存関係を解決しておく
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS publish_address_book
  DESTINATION lib/${PROJECT_NAME})

# パッケージ内のソースにメッセージの型をリンクさせる
# This CMake code is only required when you want to use interfaces in the same package
# as the one in which they are used.
rosidl_target_interfaces(publish_address_book ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
