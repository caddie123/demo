cmake_minimum_required(VERSION 3.0.2)
project(robot_ctrl)

## C++11 지원
# add_compile_options(-std=c++11)

## 필요한 Catkin 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  yolov5_ros
  geometry_msgs
  cv_bridge
  message_filters
  realsense2_description
  tf
  robot_state_publisher
  rviz
  nav_msgs
  unitree_lidar_ros
  message_generation  # 메시지 생성 필수
)

## 시스템 의존성 찾기
find_package(Eigen3 REQUIRED)

## 포함할 디렉토리 설정
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ~/unilidar_sdk/unitree_lidar_sdk/include
)

## 메시지 파일 추가
add_message_files(
   FILES
   DepthandDeg.msg
   CenterAndArray.msg
   BestPath.msg
)

## 메시지 생성
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

## Catkin 패키지 설정
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs yolov5_ros message_runtime cv_bridge message_filters
)

## 실행 파일을 위한 include 디렉토리 추가
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 실행 파일 추가
add_executable(STOP_node src/Nodes/STOP.cpp)
add_executable(path_selector src/Nodes/path_selector.cpp)

## 메시지 생성 후 의존성 추가
add_dependencies(path_selector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(STOP_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 실행 파일을 Catkin 라이브러리와 연결
target_link_libraries(STOP_node ${catkin_LIBRARIES})
target_link_libraries(path_selector ${catkin_LIBRARIES})

#############
## 설치 ##
#############

catkin_install_python(PROGRAMS
  src/Nodes/human_location_processor.py
  src/Nodes/depth_and_degree_plotter.py
  src/Nodes/model_states_plotter.py
  src/Nodes/model_location_plotter.py
  src/Nodes/location_plot.py
  src/Nodes/linear_actuator_cmd.py
  src/Nodes/set_down.py
  src/Nodes/onoff.py
  src/Nodes/human_follower.py
  src/Nodes/motor_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 실행 파일 설치
install(TARGETS STOP_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS path_selector
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

