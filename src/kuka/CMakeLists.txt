cmake_minimum_required(VERSION 3.8)
project(kuka_libs_local)

####################################
# kuka_libs_local          [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #

# Compile Kuka library separately because it has warnings that bloom does not like
add_library(${PROJECT_NAME}_static STATIC
    base/friClientApplication.cpp
    client_lbr/friLBRClient.cpp
    client_lbr/friLBRCommand.cpp
    client_lbr/friLBRState.cpp
    client_trafo/friTransformationClient.cpp
    connection/friUdpConnection.cpp
    nanopb-0.2.8/pb_decode.c
    nanopb-0.2.8/pb_encode.c
    protobuf/friCommandMessageEncoder.cpp
    protobuf/friMonitoringMessageDecoder.cpp
    protobuf/pb_frimessages_callbacks.c
    protobuf_gen/FRIMessages.pb.c
)

target_include_directories(${PROJECT_NAME}_static
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/base>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/nanopb-0.2.8>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/protobuf>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/protobuf_gen>

    )
    
target_compile_options(${PROJECT_NAME}_static
    PRIVATE
    -fPIC
    -w
)

target_compile_definitions(${PROJECT_NAME}_static
    PRIVATE
    -DHAVE_SOCKLEN_T
    -DPB_FIELD_16BIT
    -DHAVE_STDINT_H
    -DHAVE_STDDEF_H
    -DHAVE_STDBOOL_H
    -DHAVE_STDLIB_H
    -DHAVE_STRING_H
)

# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# kuka_libs_local          [END] #
##################################

