cmake_minimum_required(VERSION 3.7)
project(apriltag)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -Wunused -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_STANDARD 11)
include_directories(
        common
        include
        ${PROJECT_SOURCE_DIR}
)

find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")
set(SOURCE_FILES
        common/g2d.c
        common/getopt.c
        common/homography.c
        common/image_f32.c
        common/image_u8.c
        common/image_u8x3.c
        common/image_u8x4.c
        common/matd.c
        common/pam.c
        common/pjpeg-idct.c
        common/pjpeg.c
        common/pnm.c
        common/string_util.c
        common/svd22.c
        common/time_util.c
        common/unionfind.c
        common/workerpool.c
        common/zarray.c
        common/zhash.c
        common/zmaxheap.c

        src/apriltag.c
        src/apriltag_quad_thresh.c
        src/tag16h5.c
        src/tag25h7.c
        src/tag25h9.c
        src/tag36artoolkit.c
        src/tag36h10.c
        src/tag36h11.c
        )

add_library(apriltag ${SOURCE_FILES})

add_executable(demo
        example/apriltag_demo.c
        )
target_link_libraries(demo
        apriltag

)

add_executable(opencv_demo
        example/opencv_demo.cc
        )
target_link_libraries(opencv_demo
        ${OpenCV_LIBS}
        apriltag
        )