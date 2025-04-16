cmake_minimum_required(VERSION 3.10)
project(PeopleDetection)

# Raspberry Pi specific settings
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -mfloat-abi=hard")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a+crc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize")
endif()

# Find OpenCV package - explicit objdetect component
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn highgui videoio imgcodecs imgproc objdetect)

# Find libcamera package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Include headers
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBCAMERA_INCLUDE_DIRS})

# 显示 OpenCV 版本和包含的库信息（调试用）
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

# Add executables
add_executable(peopleDetection peopleDetection.cpp)
add_executable(camtest Camtest.cpp)
add_executable(peopleDetectionTest peopleDetectionTest.cpp)

# Link libraries - explicitly list objdetect library
target_link_libraries(peopleDetection ${OpenCV_LIBS})
target_link_libraries(camtest ${OpenCV_LIBS})
target_link_libraries(peopleDetectionTest ${OpenCV_LIBS} ${LIBCAMERA_LIBRARIES})

# Add explicit link to objdetect library
find_library(OPENCV_OBJDETECT_LIB opencv_objdetect)
if(OPENCV_OBJDETECT_LIB)
    message(STATUS "Found opencv_objdetect library: ${OPENCV_OBJDETECT_LIB}")
    target_link_libraries(peopleDetectionTest ${OPENCV_OBJDETECT_LIB})
else()
    message(WARNING "opencv_objdetect library not found explicitly")
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Raspberry Pi specific libraries
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    find_package(Threads REQUIRED)
    target_link_libraries(peopleDetection Threads::Threads)
    target_link_libraries(camtest Threads::Threads)
    target_link_libraries(peopleDetectionTest Threads::Threads)
endif()