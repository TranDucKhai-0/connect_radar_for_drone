cmake_minimum_required(VERSION 3.10)
project(connect_radar_for_drone VERSION 1.0.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Các thư mục bao gồm header files
include_directories(include)

# Lấy tất cả các file mã nguồn C++ và C
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.c")

# Chỉ định file thực thi chính
add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE MAVLINK_COMM_NUM_BUFFERS=4)

# Liên kết với các thư viện cần thiết (pthread cho đa luồng)
target_link_libraries(${PROJECT_NAME} pthread)
