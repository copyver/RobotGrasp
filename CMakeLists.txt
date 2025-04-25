cmake_minimum_required(VERSION 3.28)
project(ROBOT_GRASP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

file(GLOB_RECURSE SOURCES
        "${CMAKE_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_SOURCE_DIR}/src/*.hpp"
        "${CMAKE_SOURCE_DIR}/include/*.cpp"
)
# add 3rdParty
add_subdirectory(third_party/tysdk)

# QT
find_package(Qt5 COMPONENTS
        Core
        Widgets
        REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)
qt5_add_resources(RESOURCE_SRCS "resources/resources.qrc")

add_subdirectory(src_tycam)
add_subdirectory(src_dobot)

# python torch
set(Python3_ROOT_DIR "/home/yhlever/anaconda3/envs/maskposenet")
set(Python3_FIND_VIRTUALENV FIRST)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# numpy
set(NUMPY_INCLUDE_DIR ${Python3_ROOT_DIR}/lib/python3.10/site-packages/numpy/core/include)

# -------------- OpenCv --------------
set(OpenCV_DIR "/home/yhlever/opencv/build")
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${Python3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${NUMPY_INCLUDE_DIR}
)

add_executable(ROBOT_GRASP main.cpp ${SOURCES} ${RESOURCE_SRCS})

target_link_libraries(ROBOT_GRASP
        Qt5::Core
        Qt5::Widgets
        src_tycam_lib
        tycam
        ${OpenCV_LIBS}
        src_dobot_lib
        ${Python3_LIBRARIES}
)

