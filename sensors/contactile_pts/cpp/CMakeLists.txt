cmake_minimum_required(VERSION 3.16)
project(contactile_pts_cpp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 直接引用仓库内 vendor SDK，避免依赖容易失效的相对 symlink。
set(CONTACTILE_SDK_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../vendor/contactile_pts/C++LIN")

# 查找线程库（PTSDK 多线程示例需要）
find_package(Threads REQUIRED)

# 最简读取示例
add_executable(minimal_reader src/minimal_reader.cpp)
target_include_directories(minimal_reader PRIVATE ${CONTACTILE_SDK_DIR}/Include)
target_link_directories(minimal_reader PRIVATE ${CONTACTILE_SDK_DIR}/Library)
target_link_libraries(minimal_reader PTSDK Threads::Threads)
