cmake_minimum_required(VERSION 3.10)
project(FaceDetection)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executables
add_executable(faceEyeDetection faceEyeDetection.cpp)
add_executable(peopleDetection peopleDetection.cpp)

# Link OpenCV libraries
target_link_libraries(faceEyeDetection ${OpenCV_LIBS})
target_link_libraries(peopleDetection ${OpenCV_LIBS})

# Add model files
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
