cmake_minimum_required(VERSION 3.8)
project(pawn_orbslam3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# Set with: colcon build --cmake-args -DORB_SLAM3_DIR=/path/to/ORB_SLAM3
if(NOT DEFINED ORB_SLAM3_DIR)
    if(DEFINED ENV{ORB_SLAM3_DIR} AND NOT "$ENV{ORB_SLAM3_DIR}" STREQUAL "")
        set(ORB_SLAM3_DIR "$ENV{ORB_SLAM3_DIR}" CACHE PATH "ORB-SLAM3 source directory")
    else()
        set(ORB_SLAM3_DIR "$ENV{HOME}/PAWN/ORB_SLAM3" CACHE PATH "ORB-SLAM3 source directory")
    endif()
endif()
foreach(REQUIRED_FILE
    "${ORB_SLAM3_DIR}/include/System.h"
    "${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so"
    "${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so"
    "${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so")
    if(NOT EXISTS "${REQUIRED_FILE}")
        message(FATAL_ERROR "ORB-SLAM3 dependency missing: ${REQUIRED_FILE}. Set ORB_SLAM3_DIR and build ORB-SLAM3 first.")
    endif()
endforeach()

add_executable(orb_slam_node
    src/orb_slam_node.cpp
)

target_include_directories(orb_slam_node PRIVATE
    ${ORB_SLAM3_DIR}
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

ament_target_dependencies(orb_slam_node
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    cv_bridge
)

target_link_libraries(orb_slam_node
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
    ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen
)

# Preserve external dependency lookup after a normal (non-symlink) install.
# Paths come from this machine's configured dependency checkout, not a developer path.
set_target_properties(orb_slam_node PROPERTIES
    INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS orb_slam_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
