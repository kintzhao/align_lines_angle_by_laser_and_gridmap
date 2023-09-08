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

catkin_package( INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR})

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(extract_line_from_laser   src/laser_line/laser_feature_ros.cpp src/laser_line/line_feature.cpp)
target_link_libraries(extract_line_from_laser ${catkin_LIBRARIES}  glog)

add_library(extract_line_from_map src/laser_line/extract_line_from_map.cpp  src/laser_line/lsd.c)
target_link_libraries(extract_line_from_map ${catkin_LIBRARIES}  glog)
 
add_library(match_line  src/laser_line/match_line.cpp)
target_link_libraries(match_line ${catkin_LIBRARIES}  ceres  glog dw)

add_library(align_lines  src/laser_line/align_lines.cpp)
target_link_libraries(align_lines ${catkin_LIBRARIES}  ceres  glog dw  extract_line_from_laser  extract_line_from_map )
 
add_executable(test_extract_line_from_laser  src/test/test_extract_line_from_laser.cpp)
target_link_libraries(test_extract_line_from_laser ${catkin_LIBRARIES}  ceres  glog dw extract_line_from_laser)

add_executable(test_extract_line_from_map  src/test/test_extract_line_from_map.cpp)
target_link_libraries(test_extract_line_from_map ${catkin_LIBRARIES}  ceres  glog dw extract_line_from_map)

add_executable(test_match_line  src/test/test_match_line.cpp)
target_link_libraries(test_match_line ${catkin_LIBRARIES}  ceres  glog dw match_line)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS extract_line_from_laser extract_line_from_map  match_line  align_lines 
                test_extract_line_from_laser test_extract_line_from_map test_match_line
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 