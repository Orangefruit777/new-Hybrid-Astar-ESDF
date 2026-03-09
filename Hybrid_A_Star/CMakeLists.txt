cmake_minimum_required(VERSION 3.10)
project(hybrid_a_star)

#set(CMAKE_VERBOSE_MAKEFILE "true")

# Compiler
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_FLAGS} doesn't have C++11 support.
                         Please use a different C++ compiler")
endif ()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")



# Eigen
# include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIR})

# glog
# include(cmake/glog.cmake)
# include_directories(${GLOG_INCLUDE_DIRS})

# catkin
find_package(
    catkin REQUIRED COMPONENTS
    nav_msgs
    roscpp
    roslib
    tf
    pcl_conversions
    pcl_ros
    # tools
    # decomp_ros_utils
    tf2
    tf2_geometry_msgs
)
find_package(PCL REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

catkin_package(
     INCLUDE_DIRS include
    #  LIBRARIES path_searcher
    #  CATKIN_DEPENDS nav_mags roscpp tf
    #  DEPENDS system_lib
)

add_library(
    HYBRID_ASTAR_LIB SHARED
    src/util/rs_path.cpp
    src/util/dubins_curve.cpp
    src/util/costmap_subscriber.cpp
    src/util/goal_pose_subscriber.cpp
    src/util/init_pose_subscriber.cpp
    src/util/smooth.cpp 
    src/util/quintic_spline.cpp
    src/plan_env/sdf_map_2d.cpp
    src/Astar_path/a_star.cpp
    src/Astar_path/a_star_node.cpp

    src/hybrid_a_star.cpp
    src/hybrid_a_star_node.cpp 
    # src/hybrid_parking_opt.cpp
    src/hybrid_parking.cpp
)

target_link_libraries(
    HYBRID_ASTAR_LIB
    ${catkin_LIBRARIES}
    # ${GLOG_LIBRARIES}
    ${PCL_LIBRARIES}
)


#***run hybrid A* node***
add_executable(hybrid_astar_run app/hybrid_astar_run.cpp)
target_link_libraries(hybrid_astar_run HYBRID_ASTAR_LIB)

add_executable(parking_run app/parking_run.cpp)
target_link_libraries(parking_run HYBRID_ASTAR_LIB)

# add_executable(parking_opt app/parking_opt.cpp)
# target_link_libraries(parking_opt HYBRID_ASTAR_LIB)

#***run A* node***
add_executable(astar_run app/astar_run.cpp)
target_link_libraries(astar_run HYBRID_ASTAR_LIB)

#***run many random map***
add_executable(m_path_opt src/m_path_opt.cpp)
target_link_libraries(m_path_opt HYBRID_ASTAR_LIB)

#***map_publish***
add_executable(map_publish_y src/plan_env/map_publish_y.cpp)
target_link_libraries(map_publish_y 
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

add_executable(map_publish_parking src/plan_env/map_publish_parking.cpp)
target_link_libraries(map_publish_parking 
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

add_executable(map_random_m src/plan_env/map_random_m.cpp)
target_link_libraries(map_random_m
                        HYBRID_ASTAR_LIB
                        ${catkin_LIBRARIES}
                        ${PCL_LIBRARIES}
)

# add_executable(rear_wheel_controller controller/rear_wheel_controller.cpp)
# target_link_libraries(rear_wheel_controller 
#                         ${catkin_LIBRARIES}
# )

add_executable(lqr_run_node 
    controller/lqr_run_node.cpp
    controller/lqr_core.cpp)
target_link_libraries(lqr_run_node 
                        ${catkin_LIBRARIES}
)