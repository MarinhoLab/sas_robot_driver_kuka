cmake_minimum_required(VERSION 3.20)
project(sas_robot_driver_kuka)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(sas_conversions REQUIRED)
find_package(sas_robot_driver REQUIRED)

add_subdirectory(src/kuka)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.
add_library(${PROJECT_NAME} SHARED
    src/sas_driver_kuka.cpp

    src/joint_overlay_app.cpp
    src/joint_overlay_client.cpp
    )

target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<INSTALL_INTERFACE:include>)

target_include_directories(${PROJECT_NAME}
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka/base>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka/nanopb-0.2.8>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka/protobuf>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/kuka/protobuf_gen>

    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    kuka_libs_local_static
    PUBLIC
    sas_core::sas_core
    Eigen3::Eigen
    )

install(
    DIRECTORY include/
    DESTINATION include

    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include

    )
    
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME sas_robot_driver_kuka_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
    src/sas_robot_driver_kuka_node.cpp

    )


target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME}
    PRIVATE
    ${PROJECT_NAME}
    kuka_libs_local_static
    rclcpp::rclcpp
    sas_common::sas_common
    sas_core::sas_core
    sas_robot_driver::sas_robot_driver
    )

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_compile_features(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC c_std_99 cxx_std_17)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
    DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

ament_package()    
