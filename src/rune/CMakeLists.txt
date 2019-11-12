project(rune)
message(STATUS "Configuring sub_project: Rune")

#头文件位置
include_directories(
    ${CMAKE_SOURCR_DIR}
    /usr/local/include
    ${CMAKE_SOURCR_DIR}/include/rune
    ${CMAKE_SOURCR_DIR}/include/header
)

#添加一个库
add_library(
    rune
    Rune.cpp
    Settings.cpp
)

#添加动态链接库的位置
target_link_libraries(
    rune
    ${OpenCV_LIBRARIES}
)
