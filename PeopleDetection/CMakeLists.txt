cmake_minimum_required(VERSION 3.10)
project(FaceDetection)

# Set vcpkg toolchain for Windows
if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/Pu Fangzhe/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

# Find OpenCV package
find_package(OpenCV 4.3.0 REQUIRED COMPONENTS core dnn)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executables
add_executable(faceEyeDetection faceEyeDetection.cpp)
add_executable(peopleDetection peopleDetection.cpp)
add_executable(camtest Camtest.cpp)

# Link OpenCV libraries
target_link_libraries(faceEyeDetection ${OpenCV_LIBS})
target_link_libraries(peopleDetection ${OpenCV_LIBS})
target_link_libraries(camtest ${OpenCV_LIBS})

# Add model files
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/models DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
