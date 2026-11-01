cmake_minimum_required(VERSION 3.10)
project(yolov5_ball_live_full LANGUAGES C CXX)
 
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
 
set(VIP_LIBS NBGlinker VIPhal)
 
file(GLOB NPULIB_SRCS
"${CMAKE_CURRENT_SOURCE_DIR}/libawnn_viplite/*.c"
)
 
add_executable(yolov5_ball_live_full
src/main.cpp
src/yolov5_post_process.cpp
${NPULIB_SRCS}
)
 
target_include_directories(yolov5_ball_live_full PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/src
${CMAKE_CURRENT_SOURCE_DIR}/libawnn_viplite
${CMAKE_CURRENT_SOURCE_DIR}/libawutils
)
 
target_link_libraries(yolov5_ball_live_full
${OpenCV_LIBS}
${VIP_LIBS}
pthread
OpenCL
)

add_library(yolo_ball SHARED
    src/yolo_ball_api.cpp
    src/yolov5_post_process.cpp
    ${NPULIB_SRCS}
)

target_include_directories(yolo_ball PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/libawnn_viplite
    ${CMAKE_CURRENT_SOURCE_DIR}/libawutils
)

target_link_libraries(yolo_ball
    ${OpenCV_LIBS}
    ${VIP_LIBS}
    pthread
    OpenCL
)

set_target_properties(yolo_ball PROPERTIES
    OUTPUT_NAME yolo_ball
)
