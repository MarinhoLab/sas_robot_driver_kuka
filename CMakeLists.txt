cmake_minimum_required(VERSION 3.8)
project(sas_robot_driver_kuka)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH $ENV{QT_PATH})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sas_core REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core)

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

    src/kuka/base/friClientApplication.cpp

    src/kuka/client_lbr/friLBRClient.cpp
    src/kuka/client_lbr/friLBRCommand.cpp
    src/kuka/client_lbr/friLBRState.cpp

    src/kuka/client_trafo/friTransformationClient.cpp

    src/kuka/connection/friUdpConnection.cpp

    src/kuka/nanopb-0.2.8/pb_decode.c
    src/kuka/nanopb-0.2.8/pb_encode.c

    src/kuka/protobuf/friCommandMessageEncoder.cpp
    src/kuka/protobuf/friMonitoringMessageDecoder.cpp
    src/kuka/protobuf/pb_frimessages_callbacks.c

    src/kuka/protobuf_gen/FRIMessages.pb.c
    )

ament_target_dependencies(${PROJECT_NAME}
    sas_core
    Eigen3
    Qt6Core

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

# Defines related to the kuka libraries
target_compile_definitions(${PROJECT_NAME}
    PRIVATE

    -DHAVE_SOCKLEN_T
    -DPB_FIELD_16BIT
    -DHAVE_STDINT_H
    -DHAVE_STDDEF_H
    -DHAVE_STDBOOL_H
    -DHAVE_STDLIB_H
    -DHAVE_STRING_H
)



ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Eigen3
    sas_core
    Qt6::Core

    )

target_link_libraries(${PROJECT_NAME}
    Qt6::Core

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
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()    
