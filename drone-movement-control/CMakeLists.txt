cmake_minimum_required(VERSION 3.10)
project(drone_controller CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置编译选项
add_compile_options(-Wall -Wextra -pedantic)

# 包含目录
include_directories(include)

# 寻找线程库
find_package(Threads REQUIRED)

# 添加所有源文件(不包括测试主文件)
file(GLOB SOURCES "src/*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/callback_test_main.cpp")

# 创建主要可执行文件
add_executable(drone_controller ${SOURCES})

# 创建回调测试可执行文件
add_executable(callback_test_app src/callback_test.cpp src/callback_test_main.cpp)

# 链接线程库
target_link_libraries(drone_controller Threads::Threads)
target_link_libraries(callback_test_app Threads::Threads)

# 安装目标
install(TARGETS drone_controller DESTINATION bin)
install(TARGETS callback_test_app DESTINATION bin) 