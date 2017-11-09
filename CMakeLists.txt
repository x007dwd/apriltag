cmake_minimum_required(VERSION 2.8)
project(apriltag)

SET(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
#set(CMAKE_CXX_STANDARD 11)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake-modules)

set(DUO_SDK ${PROJECT_SOURCE_DIR}/example/SDK)
include_directories(${DUO_SDK}/include)
if (CMAKE_SYSTEM_PROCESSOR STREQUAL armv7l OR CMAKE_SYSTEM_PROCESSOR STREQUAL aarch64)
    set(DUOLib_ARCH arm)
else ()
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(DUOLib_ARCH x64)
    else ()
        set(DUOLib_ARCH x86)
    endif ()
endif ()
find_library(DUO_LIBRARY DUO HINTS ${DUO_SDK}/linux/${DUOLib_ARCH})

find_package( Glog REQUIRED )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
        common
        include
        example
        pose
        ${PROJECT_SOURCE_DIR}
        ${GLOG_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
)

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

add_executable(opencv_demo
        example/opencv_demo.cc
        )
target_link_libraries(opencv_demo
        ${OpenCV_LIBS}
        apriltag
        )

add_executable(duo_pose
        example/DuoAprilTag.cpp
        example/DUOReader.cpp
        pose/PoseEstimate.cpp
        )
target_link_libraries(duo_pose
        ${DUO_LIBRARY}
        apriltag
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${EIGEN3_LIBS}
        ${BLAS_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${Pangolin_LIBRARIES}
        g2o_core g2o_stuff g2o_solver_csparse g2o_types_sba g2o_csparse_extension
        ${CSPARSE_LIBRARY}
)

