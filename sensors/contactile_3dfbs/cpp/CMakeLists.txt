cmake_minimum_required(VERSION 3.10)
project(minimal_reader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 直接引用仓库内 vendor SDK，避免依赖相对 symlink。
set(CONTACTILE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/contactile_3dfbs/C++LIN")

add_executable(minimal_reader src/minimal_reader.cpp)
target_include_directories(minimal_reader PRIVATE ${CONTACTILE_SDK_DIR}/Include)
target_link_directories(minimal_reader PRIVATE ${CONTACTILE_SDK_DIR}/Library)
target_link_libraries(minimal_reader libPTSDK.a pthread)
