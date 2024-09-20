cmake_minimum_required(VERSION 3.10)
project(warpaffine LANGUAGES CUDA CXX)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
set(CMAKE_BUILD_TYPE "DEBUG")

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.cu")

add_executable(warp_affine_demo ${SOURCES})

target_link_libraries(warp_affine_demo ${CUDA_LIBRARIES} ${OpenCV_LIBS})


