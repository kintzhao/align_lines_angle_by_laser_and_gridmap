cmake_minimum_required(VERSION 2.8.3)
project(laser_line)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs    
    std_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    message_generation        
)

find_package(glog) 
# 寻找Ceres库
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
        Line.msg Lines.msg   
)

generate_messages(DEPENDENCIES geometry_msgs   std_msgs)

catkin_package( INCLUDE_DIRS include)
 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(extract_line_from_laser src/extract_line_from_laser.cpp  src/laser_feature_ros.cpp src/line_feature.cpp)
target_link_libraries(extract_line_from_laser ${catkin_LIBRARIES}  glog)

add_executable(extract_line_from_map src/extract_line_from_map.cpp  src/lsd.c)
target_link_libraries(extract_line_from_map ${catkin_LIBRARIES}  glog)
 
add_executable(match_line_node  src/match_line_node.cpp)
target_link_libraries(match_line_node ${catkin_LIBRARIES}  ceres  glog dw)
target_include_directories(match_line_node PRIVATE ${EIGEN3_INCLUDE_DIR})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS extract_line_from_laser extract_line_from_map  match_line_node
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 