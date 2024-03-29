cmake_minimum_required(VERSION 2.8.3)
project(bluecougar)
set(PROJECT_NAME bluecougar)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  cv_bridge

  sensor_msgs
  std_msgs
  message_filters
  image_transport

    camera_info_manager
  diagnostic_updater
)
generate_dynamic_reconfigure_options(
  cfg/bluecougarDyn.cfg
)

catkin_package(
	DEPENDS Eigen
  CATKIN_DEPENDS 
)

set(mvIMPACT_DIR /opt/mvIMPACT_Acquire)
set(mvIMPACT_LIBRARIES_DIR /opt/mvIMPACT_Acquire/lib/x86_64)

link_directories(
	${mvIMPACT_LIBRARIES_DIR}
)
link_libraries(
	mvDeviceManager 
	mvImageProcessing 
	mvIMPACT_Acquire.java
	mvPropHandling  
	usb-1.0
)



include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Eigen_INCLUDE_DIRS}
  ${mvIMPACT_DIR}
 )

set(SRC ${SRC} src/main_hhi.cpp)

set(DCMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg -O2 -m64 -Ofast -flto -msse3 -march=native -mfpmath=sse")

add_executable (bluecougar_multiple_node src/main_hhi.cpp )
add_dependencies(bluecougar_multiple_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries (bluecougar_multiple_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} )
