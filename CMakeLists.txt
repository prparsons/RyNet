# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

# declare my-recognition project
project(overallApp)

add_subdirectory(threadLib)

# import jetson-inference and jetson-utils packages.
# note that if you didn't do "sudo make install"
# while building jetson-inference, this will error.
find_package(jetson-utils)
find_package(jetson-inference)

# Realsense and OpenCV
find_package( realsense2 REQUIRED )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(/usr/lib/aarch64-linux-gnu/tegra)

# OpenSSL
find_package(OpenSSL REQUIRED)

# CUDA and Qt4 are required
find_package(CUDA)
find_package(Qt4)

# setup Qt4 for build
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

#was getting a missing lnvbuf_util linker error
link_directories(/usr/lib/aarch64-linux-gnu/tegra) 

# compiler flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# compile the my-recognition program
cuda_add_executable(overallApp overallApp.cpp dataLogger.cpp cameraMulti.cpp planner.cpp navigation.cpp route.cpp )

# link my-recognition to jetson-inference library
target_link_libraries(overallApp jetson-inference realsense2 OpenSSL::SSL ${OpenCV_LIBS})
