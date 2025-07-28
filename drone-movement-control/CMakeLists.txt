cmake_minimum_required(VERSION 3.10)
project(BalanceDroneRealTime C CXX)

# Set the C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for real-time performance
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# Set compile options
add_compile_options(-Wall -Wextra -pedantic)

# Include directories - updated after removing empty directories
include_directories(inc inc/control_classes inc/dmp)

# Find the Threads package
find_package(Threads REQUIRED)

# Find libgpiod package - require v2.0+ with new API
find_package(PkgConfig REQUIRED)

# 要求至少 libgpiod v2.0，确保有新的API（gpiod_line_request_release等）
pkg_check_modules(GPIOD REQUIRED libgpiod>=2.0)
message(STATUS "→ Found libgpiod v${GPIOD_VERSION}")
message(STATUS "  Include dirs: ${GPIOD_INCLUDE_DIRS}")
message(STATUS "  Library dirs: ${GPIOD_LIBRARY_DIRS}")
message(STATUS "  Libraries: ${GPIOD_LIBRARIES}")
message(STATUS "  CFLAGS: ${GPIOD_CFLAGS}")

# Add all source files (excluding test files and problematic DMP files) - updated file extensions
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/control_classes/*.cpp")
# Temporarily exclude DMP files due to STM32-specific dependencies
# file(GLOB_RECURSE DMP_SOURCES "src/dmp/*.cpp")
file(GLOB_RECURSE HEADERS "inc/*.hpp" "inc/*.h")

# Remove test files and DMP files from main sources
list(FILTER SOURCES EXCLUDE REGEX ".*test.*")
list(FILTER SOURCES EXCLUDE REGEX ".*/dmp/.*")

# libgpiod v2+ is required, so always use real GPIO implementation
message(STATUS "Using libgpiod v2+ GPIO implementation")
# Remove stub file if it somehow got included in sources
list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/GPIOHelper_stub.cpp")

# Create the main executable
add_executable(balance_drone ${SOURCES} ${HEADERS})

# Configure libgpiod for main executable
target_include_directories(balance_drone PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_directories(balance_drone PRIVATE ${GPIOD_LIBRARY_DIRS})
target_compile_options(balance_drone PRIVATE ${GPIOD_CFLAGS})

# Create test executable
file(GLOB TEST_SOURCES "tests/*.cpp")
set(TEST_SOURCES_FILTERED ${SOURCES})
list(REMOVE_ITEM TEST_SOURCES_FILTERED "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
add_executable(balance_drone_tests ${TEST_SOURCES} ${TEST_SOURCES_FILTERED})

# Configure libgpiod for test executable
target_include_directories(balance_drone_tests PRIVATE ${GPIOD_INCLUDE_DIRS})
target_link_directories(balance_drone_tests PRIVATE ${GPIOD_LIBRARY_DIRS})
target_compile_options(balance_drone_tests PRIVATE ${GPIOD_CFLAGS})

# Link libraries - libgpiod v2+ is required
target_link_libraries(balance_drone
    PRIVATE
        Threads::Threads
        rt
        ${GPIOD_LIBRARIES}
)

target_link_libraries(balance_drone_tests
    PRIVATE
        Threads::Threads
        rt
        ${GPIOD_LIBRARIES}
)

message(STATUS "Linking with libgpiod v${GPIOD_VERSION} support")

# Enable testing
enable_testing()
add_test(NAME balance_system_tests COMMAND balance_drone_tests)

# Custom targets
add_custom_target(run_tests
    COMMAND ${CMAKE_BINARY_DIR}/balance_drone_tests
    DEPENDS balance_drone_tests
    COMMENT "Running all tests..."
)

add_custom_target(performance_test
    COMMAND ${CMAKE_BINARY_DIR}/balance_drone_tests --performance
    DEPENDS balance_drone_tests
    COMMENT "Running performance tests..."
)

# Legacy callback test executable for compatibility
add_executable(callback_test_app callback_test.cpp callback_test_main.cpp)
target_link_libraries(callback_test_app Threads::Threads)

# Install targets
install(TARGETS balance_drone DESTINATION bin)
install(TARGETS balance_drone_tests DESTINATION bin)
install(TARGETS callback_test_app DESTINATION bin)
install(FILES README.md DESTINATION share/doc/balance_drone)

# Packaging
set(CPACK_PACKAGE_NAME "RealTimeBalanceDrone")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION "ENG5220 Real-time Balance Drone System")
include(CPack)
