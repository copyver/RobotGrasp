project(src_dobot)

set(CMAKE_AUTOMOC ON)

file(GLOB SRC_DOBOT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.cpp")

find_package(Qt5 COMPONENTS Core REQUIRED)

add_library(src_dobot_lib STATIC ${SRC_DOBOT_SOURCES})

target_include_directories(src_dobot_lib PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(src_dobot_lib
        Qt5::Core
)