cmake_minimum_required(VERSION 3.10)
project(PeopleDetection)

# Raspberry Pi specific settings
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -mfloat-abi=hard")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a+crc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize")
endif()

# Find OpenCV package
find_package(OpenCV 4.9.0 REQUIRED COMPONENTS core dnn highgui videoio imgcodecs)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executables
add_executable(peopleDetection peopleDetection.cpp)
add_executable(camtest Camtest.cpp)

# Link OpenCV libraries
target_link_libraries(peopleDetection ${OpenCV_LIBS})
target_link_libraries(camtest ${OpenCV_LIBS})

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Raspberry Pi specific libraries
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    find_package(Threads REQUIRED)
    target_link_libraries(peopleDetection Threads::Threads)
    target_link_libraries(camtest Threads::Threads)
endif()
