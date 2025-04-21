cmake_minimum_required(VERSION 3.10)
project(PeopleDetectionStream)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

# If OpenSSL is needed
find_package(OpenSSL)
if(OPENSSL_FOUND)
  add_definitions(-DCPPHTTPLIB_OPENSSL_SUPPORT)
  include_directories(${OPENSSL_INCLUDE_DIR})
endif()

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${LIBCAMERA_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add third-party header-only library
add_subdirectory(third_party)

# Add source files
set(SOURCE_FILES 
  peopleDetectionStream.cpp
)

# Create the executable
add_executable(peopleDetectionStream ${SOURCE_FILES})

# Link libraries
target_link_libraries(peopleDetectionStream
  ${OpenCV_LIBS}
  ${LIBCAMERA_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# If OpenSSL is found, link it
if(OPENSSL_FOUND)
  target_link_libraries(peopleDetectionStream ${OPENSSL_LIBRARIES})
endif()

# Install targets
install(TARGETS peopleDetectionStream DESTINATION bin)

# Install public directory for web files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/public/ DESTINATION share/peopleDetectionStream/public)

# Copy the public directory to build directory for testing
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/public/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/public/)

# Add custom command to install httplib
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_custom_command(
  TARGET peopleDetectionStream PRE_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/include
  COMMAND curl -o ${CMAKE_CURRENT_SOURCE_DIR}/include/httplib.h -L https://raw.githubusercontent.com/yhirose/cpp-httplib/v0.12.0/httplib.h
  COMMENT "Downloading cpp-httplib"
)