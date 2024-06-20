cmake_minimum_required(VERSION 3.5)
project(fsr_experiments)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
message("current binary_dir: " ${CMAKE_CURRENT_BINARY_DIR})
message("current source_dir: " ${CMAKE_CURRENT_SOURCE_DIR})
set(ProtobufIncludePath ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "Path to generated protobuf files.")
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(exp_proto STATIC ${ProtoSources} ${ProtoHeaders})

target_link_libraries(exp_proto ${Protobuf_LIBRARIES})
