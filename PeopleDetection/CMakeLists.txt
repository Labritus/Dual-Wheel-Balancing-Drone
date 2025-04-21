cmake_minimum_required(VERSION 3.10)
project(PeopleDetection)

# Raspberry Pi specific settings
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -mfloat-abi=hard")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a+crc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize")
endif()

# Find package required components including explicitly objdetect
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn highgui videoio imgcodecs imgproc objdetect)

# Find libcamera package
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# Debug information
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "libcamera include dirs: ${LIBCAMERA_INCLUDE_DIRS}")
message(STATUS "libcamera libraries: ${LIBCAMERA_LIBRARIES}")

# Include headers
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${LIBCAMERA_INCLUDE_DIRS})

# Add executables
add_executable(peopleDetection peopleDetection.cpp)
add_executable(camtest Camtest.cpp)
add_executable(peopleDetectionTest peopleDetectionTest.cpp)
add_executable(peopleDetectionWebSocket peopleDetectionWebSocket.cpp)

# Link libraries with correct names
target_link_libraries(peopleDetection 
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    camera
    camera-base
)

target_link_libraries(camtest 
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    camera
    camera-base
)

target_link_libraries(peopleDetectionTest 
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    camera
    camera-base
)

target_link_libraries(peopleDetectionWebSocket 
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
    camera
    camera-base
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Raspberry Pi specific libraries
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    find_package(Threads REQUIRED)
    target_link_libraries(peopleDetection Threads::Threads)
    target_link_libraries(camtest Threads::Threads)
    target_link_libraries(peopleDetectionTest Threads::Threads)
    target_link_libraries(peopleDetectionWebSocket Threads::Threads)
endif()

# Install targets
install(TARGETS peopleDetection camtest peopleDetectionTest peopleDetectionWebSocket
    RUNTIME DESTINATION bin
)

# Install web interface files
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/web/index.html
    DESTINATION share/peopledetection/web
)

# Install Python WebSocket server
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/detection_server.py
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    DESTINATION bin
)