cmake_minimum_required(VERSION 3.10)
project(drone_controller CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set compile options
add_compile_options(-Wall -Wextra -pedantic)

# Include directories
include_directories(include)

# Find the Threads package
find_package(Threads REQUIRED)

# Add all source files (excluding the test main file)
file(GLOB SOURCES "src/*.cpp")
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/callback_test_main.cpp")

# Create the main executable
add_executable(drone_controller ${SOURCES})

# Create the callback test executable
add_executable(callback_test_app src/callback_test.cpp src/callback_test_main.cpp)

# Link the Threads library
target_link_libraries(drone_controller Threads::Threads)
target_link_libraries(callback_test_app Threads::Threads)

# Install targets
install(TARGETS drone_controller DESTINATION bin)
install(TARGETS callback_test_app DESTINATION bin)
