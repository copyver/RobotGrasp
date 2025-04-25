project(src_tycam)

set(OpenCV_DIR "/home/yhlever/opencv/build")
find_package(OpenCV REQUIRED)

file(GLOB SRC_TYCAM_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.cpp")

add_library(src_tycam_lib STATIC ${SRC_TYCAM_SOURCES})

target_include_directories(src_tycam_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/third_party/tysdk/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(src_tycam_lib PRIVATE
        tycam
        ${OpenCV_LIBS}
)
